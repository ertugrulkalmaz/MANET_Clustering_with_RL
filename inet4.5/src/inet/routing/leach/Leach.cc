#include "inet/common/IProtocolRegistrationListener.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/ProtocolTag_m.h"
#include "inet/linklayer/common/InterfaceTag_m.h"
#include "inet/networklayer/common/L3AddressTag_m.h"
#include "inet/routing/leach/Leach.h"
#include "inet/physicallayer/wireless/common/contract/packetlevel/SignalTag_m.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/power/storage/SimpleEpEnergyStorage.h"
#include "inet/mobility/contract/IMobility.h"
#include "inet/common/geometry/common/Coord.h"
#include "inet/networklayer/ipv4/Ipv4InterfaceData.h"
#include <list>
#include <vector>
#include <algorithm>
#include <ctime>
#include <functional>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cctype>
using namespace power;
namespace inet {

Define_Module(Leach);

Leach::ForwardEntry::~ForwardEntry() {
    if (this->event != nullptr) delete this->event;
    if (this->hello != nullptr) delete this->hello;
}

Leach::Leach() : event(nullptr), forwardEntry(nullptr) {}

Leach::~Leach() {
    stop();
    delete event;
}

void Leach::initialize(int stage) {
    RoutingProtocolBase::initialize(stage);

    if (stage == INITSTAGE_LOCAL) {
        sequencenumber = 0;
        host = getContainingNode(this);
        ift = getModuleFromPar<IInterfaceTable>(par("interfaceTableModule"), this);

        clusterHeadPercentage = par("clusterHeadPercentage");
        numNodes = par("numNodes");

        dataPktSent = 0;
        dataPktReceived = 0;
        dataPktReceivedVerf = 0;
        controlPktSent = 0;
        controlPktReceived = 0;
        bsPktSent = 0;
        totalChCount = 0;

        dataPktSendDelay = uniform(0, 10);
        CHPktSendDelay = par("CHPktSendDelay");
        roundDuration = par("roundDuration");

        TDMADelayCounter = 1;

        helloInterval = par("helloInterval");
        event = new cMessage("event");

        // Q-Learning parameters
        useQLearning = par("useQLearning").boolValue();
        learningAlgorithm = par("learningAlgorithm").stdstringValue();
        qLearningAlpha = par("qLearningAlpha");
        qLearningGamma = par("qLearningGamma");
        qLearningEpsilon = par("qLearningEpsilon");
        qLearningEpsilonDecay = par("qLearningEpsilonDecay");
        qLearningMinEpsilon = par("qLearningMinEpsilon");
        qTableInitRandom = par("qTableInitRandom").boolValue();
        qTableInitMin = par("qTableInitMin");
        qTableInitMax = par("qTableInitMax");
        
        // Q-table persistence parameters
        saveQTable = par("saveQTable").boolValue();
        loadQTable = par("loadQTable").boolValue();
        perNodeQTableFile = par("perNodeQTableFile").boolValue();
        qTableFilePath = resolveQTableFilePath(par("qTableFilePath").stdstringValue());
        if (!perNodeQTableFile && numNodes > 1) {
            EV << "Warning: Shared Q-table file is enabled with multiple nodes; "
               << "updates are not merged and only node index 0 saves." << endl;
        }
        
        // Initialize Q-learning components
        learningRate = qLearningAlpha;
        discountFactor = qLearningGamma;
        explorationRate = qLearningEpsilon;
        explorationDecay = qLearningEpsilonDecay;
        minExplorationRate = qLearningMinEpsilon;
        
        hasPreviousState = false;
        previousEnergy = 0.0;
        previousDataPktSent = 0;
        previousDataPktReceived = 0;
        previousRoundStartTime = 0;
        lastRoundChAdverts = 0;
        currentRoundChAdvertsSet.clear();
        
        // Initialize Q-table
        if (useQLearning) {
            initializeQTable();
            
            // Load Q-table if enabled
            if (loadQTable) {
                loadQTableFromFile(qTableFilePath);
            }
        }

        WATCH(threshold);
        WATCH(round);
        WATCH(totalChCount);
        WATCH(explorationRate);

        round = 0;
        weight = 0;
        wasCH = false;
        leachState = nch;
        roundStartTime = 0; // Initialize member variable

        // Reserve memory for vectors
        nodeMemory.reserve(numNodes);
        nodeCHMemory.reserve(numNodes);
        extractedTDMASchedule.reserve(numNodes);
    } else if (stage == INITSTAGE_ROUTING_PROTOCOLS) {
        registerService(Protocol::manet, gate("ipOut"), gate("ipIn"));
        registerProtocol(Protocol::manet, gate("ipOut"), gate("ipIn"));
    }
}
void Leach::start() {
    addToNodePosList();

    int num_802154 = 0;
    NetworkInterface *ie;
    NetworkInterface *i_face;
    const char *name;
    broadcastDelay = &par("broadcastDelay");
    for (int i = 0; i < ift->getNumInterfaces(); i++) {
        ie = ift->getInterface(i);
        name = ie->getInterfaceName();
        if (strstr(name, "wlan") != nullptr) {
            i_face = ie;
            num_802154++;
            interfaceId = i;
        }
    }

    if (num_802154 == 1) {
        interface80211ptr = i_face;
    } else {
        throw cRuntimeError("LEACH has found %i 802.15.4 interfaces", num_802154);
    }
    interface80211ptr->getProtocolDataForUpdate<Ipv4InterfaceData>()->joinMulticastGroup(Ipv4Address::LL_MANET_ROUTERS);

    event->setKind(SELF);
    scheduleAt(simTime() + uniform(0.0, par("maxVariance").doubleValue()), event);
}

void Leach::stop() {
    cancelEvent(event);
    nodeMemory.clear();
    nodeCHMemory.clear();
    extractedTDMASchedule.clear();
    TDMADelayCounter = 1;
    setLeachState(nch);
}

void Leach::handleMessageWhenUp(cMessage *msg) {
    if (msg->isSelfMessage()) {
        // Finalize neighbor count for the round that just ended
        lastRoundChAdverts = (int)currentRoundChAdvertsSet.size();
        currentRoundChAdvertsSet.clear();

        // Timeout check for CHs
                if (leachState == ch && simTime() >= roundStartTime + roundDuration) {
                    EV << "Node " << host->getFullName() << " CH timeout, reverting to NCH" << endl;
                    setLeachState(nch);
                }
                // Only revert if it's time for a new election
                else if (msg->getKind() == SELF && leachState == ch) {
                    setLeachState(nch);
                }

        // Q-Learning based CH election
        bool shouldBecomeCH = false;
        
        if (useQLearning) {
            // Get current state
            QLearningState currentState = getCurrentState();
            
            // Select action using epsilon-greedy policy for current state
            bool currentAction = selectAction(currentState);

            // Update Q-value from previous round if we have previous state
            if (hasPreviousState) {
                double reward = calculateReward();
                // SARSA uses the current action for the next state; Q-learning ignores it
                updateQValue(previousState, previousAction, reward, currentState, currentAction);
                EV << "Node " << host->getFullName() << " " << learningAlgorithm << " update: reward=" << reward
                   << ", state=" << stateToString(currentState) << endl;
            }

            shouldBecomeCH = currentAction;
            
            // Store current state and action for next round
            previousState = currentState;
            previousAction = shouldBecomeCH;
            hasPreviousState = true;
            
            // Decay exploration rate
            explorationRate = std::max(minExplorationRate, explorationRate * explorationDecay);
            
            EV << "Node " << host->getFullName() << " Q-Learning decision: " 
               << (shouldBecomeCH ? "BECOME_CH" : "REMAIN_NCH") 
               << ", epsilon=" << explorationRate << endl;
        } else {
            // Original probabilistic approach
            double randNo = uniform(0, 1);
            threshold = generateThresholdValue(round);
            shouldBecomeCH = (randNo < threshold && !wasCH);
        }

        if (shouldBecomeCH && !wasCH) {
            weight++;
            setLeachState(ch);
            wasCH = true;
            handleSelfMessage(msg);
        }

        round++;
        int intervalLength = 1.0 / clusterHeadPercentage;
        if (fmod(round, intervalLength) == 0) {
            wasCH = false;
            nodeMemory.clear();
            nodeCHMemory.clear();
            extractedTDMASchedule.clear();
            TDMADelayCounter = 1;
        }

        // Log CH count for debugging
                static int chCount = 0;
                if (leachState == ch) chCount++;

                // Move the reset to the start of a new round
                if (fmod(round, intervalLength) == 0) {
                    EV << "End of interval, resetting CH count from " << chCount << endl;
                    chCount = 0;
                }

                EV << "Round " << round << ": Total CHs = " << chCount << " at " << simTime() << endl;

        // Store previous round metrics for reward calculation
        previousEnergy = getEnergyPercentage();
        previousDataPktSent = dataPktSent;
        previousDataPktReceived = dataPktReceived;
        previousRoundStartTime = roundStartTime;

        roundStartTime = simTime();
        event->setKind(SELF);
        scheduleAt(simTime() + roundDuration, event);
    } else if (check_and_cast<Packet *>(msg)->getTag<PacketProtocolTag>()->getProtocol() == &Protocol::manet) {
        processMessage(msg);
    } else {
        EV_ERROR << "Message Not Supported:" << msg->getName() << simTime() << endl;
        //throw cRuntimeError("Message not supported %s", msg->getName());
    }
    refreshDisplay();
}

void Leach::handleSelfMessage(cMessage *msg) {
    if (msg == event && event->getKind() == SELF) {
        auto ctrlPkt = makeShared<LeachControlPkt>();
        ctrlPkt->setPacketType(CH);
        Ipv4Address source = interface80211ptr->getProtocolData<Ipv4InterfaceData>()->getIPAddress();
        ctrlPkt->setChunkLength(b(128));
        ctrlPkt->setSrcAddress(source);

        auto packet = new Packet("LEACHControlPkt", ctrlPkt);
        auto addressReq = packet->addTag<L3AddressReq>();
        addressReq->setDestAddress(Ipv4Address(255, 255, 255, 255));
        addressReq->setSrcAddress(source);
        packet->addTag<InterfaceReq>()->setInterfaceId(interface80211ptr->getInterfaceId());
        packet->addTag<PacketProtocolTag>()->setProtocol(&Protocol::manet);
        packet->addTag<DispatchProtocolReq>()->setProtocol(&Protocol::ipv4);

        send(packet, "ipOut");
        addToEventLog(source, Ipv4Address(255, 255, 255, 255), "CTRL", "SENT");
        controlPktSent++;
        bubble("Sending new enrolment message");
    } else {
        delete msg;
    }
}

void Leach::processMessage(cMessage *msg) {
    Ipv4Address selfAddr = interface80211ptr->getProtocolData<Ipv4InterfaceData>()->getIPAddress();
    auto receivedCtrlPkt = staticPtrCast<LeachControlPkt>(check_and_cast<Packet *>(msg)->peekData<LeachControlPkt>()->dupShared());
    Packet *receivedPkt = check_and_cast<Packet *>(msg);
    auto& leachControlPkt = receivedPkt->popAtFront<LeachControlPkt>();

    auto packetType = leachControlPkt->getPacketType();

    if (msg->arrivedOn("ipIn")) {
        if (packetType == CH) {
            controlPktReceived++;
            Ipv4Address CHAddr = receivedCtrlPkt->getSrcAddress();
            currentRoundChAdvertsSet.insert(CHAddr);
            addToEventLog(CHAddr, selfAddr, "CTRL", "REC");

            auto signalPowerInd = receivedPkt->getTag<SignalPowerInd>();
            double rxPower = signalPowerInd->getPower().get();

            addToNodeMemory(selfAddr, CHAddr, rxPower);
            sendAckToCH(selfAddr, CHAddr);
        } else if (packetType == ACK && leachState == ch) {
            Ipv4Address nodeAddr = receivedCtrlPkt->getSrcAddress();
            addToEventLog(nodeAddr, selfAddr, "ACK", "REC");

            addToNodeCHMemory(nodeAddr);
            EV << "CH " << host->getFullName() << " nodeCHMemory size: " << nodeCHMemory.size() << endl;
            if (nodeCHMemory.size() >= 1) { // Lowered threshold
                sendSchToNCH(selfAddr);
            }
        } else if (packetType == SCH) {
            Ipv4Address CHAddr = receivedCtrlPkt->getSrcAddress();
            addToEventLog(CHAddr, selfAddr, "SCH", "REC");

            int scheduleArraySize = receivedCtrlPkt->getScheduleArraySize();
            for (int counter = 0; counter < scheduleArraySize; counter++) {
                ScheduleEntry tempScheduleEntry = receivedCtrlPkt->getSchedule(counter);
                TDMAScheduleEntry extractedTDMAScheduleEntry;
                extractedTDMAScheduleEntry.nodeAddress = tempScheduleEntry.getNodeAddress();
                extractedTDMAScheduleEntry.TDMAdelay = tempScheduleEntry.getTDMAdelay();
                extractedTDMASchedule.push_back(extractedTDMAScheduleEntry);
            }

            double receivedTDMADelay = -1;
            for (auto& it : extractedTDMASchedule) {
                if (it.nodeAddress == selfAddr) {
                    receivedTDMADelay = it.TDMAdelay;
                    break;
                }
            }

            if (receivedTDMADelay > -1) {
                sendDataToCH(selfAddr, CHAddr, receivedTDMADelay);
            }
        } else if (packetType == DATA) {
            Ipv4Address NCHAddr = receivedCtrlPkt->getSrcAddress();
            addToEventLog(NCHAddr, selfAddr, "DATA", "REC");
            std::string fingerprint = receivedCtrlPkt->getFingerprint();

            if (checkFingerprint(fingerprint)) {
                dataPktReceivedVerf++;
            }
            dataPktReceived++;
            sendDataToBS(selfAddr, fingerprint);
        } else if (packetType == BS) {
            delete msg;
        }
    } else {
        throw cRuntimeError("Message arrived on unknown gate %s", msg->getArrivalGate()->getName());
    }
}

void Leach::handleStopOperation(LifecycleOperation *operation) {
    cancelEvent(event);
}

void Leach::handleCrashOperation(LifecycleOperation *operation) {
    cancelEvent(event);
}

double Leach::generateThresholdValue(int round) {
    int intervalLength = 1.0 / clusterHeadPercentage;
    double thresholdVal = clusterHeadPercentage / (1 - clusterHeadPercentage * fmod(round, intervalLength));
    if (thresholdVal >= 1) {
        round = 0;
    }
    return thresholdVal;
}

// Add checks when adding elements:
void Leach::addToNodeMemory(Ipv4Address nodeAddr, Ipv4Address CHAddr, double energy) {
    if (!isCHAddedInMemory(CHAddr)) {
        if (nodeMemory.size() >= nodeMemory.capacity()) {
            EV << "Warning: nodeMemory exceeding reserved capacity" << endl;
        }
        nodeMemoryObject node;
        node.nodeAddr = nodeAddr;
        node.CHAddr = CHAddr;
        node.energy = energy;
        nodeMemory.push_back(node);
    }
}

void Leach::addToNodeCHMemory(Ipv4Address NCHAddr) {
    if (!isNCHAddedInCHMemory(NCHAddr)) {
        TDMAScheduleEntry scheduleEntry;
        scheduleEntry.nodeAddress = NCHAddr;
        scheduleEntry.TDMAdelay = TDMADelayCounter;
        nodeCHMemory.push_back(scheduleEntry);
        TDMADelayCounter++;
    }
}

bool Leach::isCHAddedInMemory(Ipv4Address CHAddr) {
    for (auto& it : nodeMemory) {
        if (it.CHAddr == CHAddr) return true;
    }
    return false;
}

bool Leach::isNCHAddedInCHMemory(Ipv4Address NCHAddr) {
    for (auto& it : nodeCHMemory) {
        if (it.nodeAddress == NCHAddr) return true;
    }
    return false;
}

void Leach::generateTDMASchedule() {
    // Clear previous schedule
    nodeCHMemory.clear();
    TDMADelayCounter = 1.0;

    // Get all nodes that have acknowledged this CH
    for (auto& node : nodeMemory) {
        if (node.CHAddr == interface80211ptr->getProtocolData<Ipv4InterfaceData>()->getIPAddress()) {
            TDMAScheduleEntry scheduleEntry;
            scheduleEntry.nodeAddress = node.nodeAddr;
            scheduleEntry.TDMAdelay = TDMADelayCounter;
            nodeCHMemory.push_back(scheduleEntry);
            TDMADelayCounter++;
        }
    }

    EV << "Generated TDMA schedule with " << nodeCHMemory.size() << " slots" << endl;
}

void Leach::setLeachState(LeachState ls) {
    EV << "Node " << host->getFullName() << " state: "
       << (leachState == ch ? "CH" : "NCH") << " -> " << (ls == ch ? "CH" : "NCH")
       << " at " << simTime() << endl;
    leachState = ls;
    refreshDisplay();
}

void Leach::sendAckToCH(Ipv4Address nodeAddr, Ipv4Address CHAddr) {
    auto ackPkt = makeShared<LeachAckPkt>();
    ackPkt->setPacketType(ACK);
    ackPkt->setChunkLength(b(128));
    ackPkt->setSrcAddress(nodeAddr);

    auto ackPacket = new Packet("LeachAckPkt", ackPkt);
    auto addressReq = ackPacket->addTag<L3AddressReq>();
    addressReq->setDestAddress(getIdealCH(nodeAddr));
    addressReq->setSrcAddress(nodeAddr);
    ackPacket->addTag<InterfaceReq>()->setInterfaceId(interface80211ptr->getInterfaceId());
    ackPacket->addTag<PacketProtocolTag>()->setProtocol(&Protocol::manet);
    ackPacket->addTag<DispatchProtocolReq>()->setProtocol(&Protocol::ipv4);

    send(ackPacket, "ipOut");
    addToEventLog(nodeAddr, getIdealCH(nodeAddr), "ACK", "SENT");
}

void Leach::sendSchToNCH(Ipv4Address selfAddr) {
    auto schedulePkt = makeShared<LeachSchedulePkt>();
    schedulePkt->setPacketType(SCH);
    schedulePkt->setChunkLength(b(128));
    schedulePkt->setSrcAddress(selfAddr);

    for (auto& it : nodeCHMemory) {
        ScheduleEntry scheduleEntry;
        scheduleEntry.setNodeAddress(it.nodeAddress);
        scheduleEntry.setTDMAdelay(it.TDMAdelay);
        schedulePkt->appendSchedule(scheduleEntry);
    }

    auto schedulePacket = new Packet("LeachSchedulePkt", schedulePkt);
    auto scheduleReq = schedulePacket->addTag<L3AddressReq>();
    scheduleReq->setDestAddress(Ipv4Address(255, 255, 255, 255));
    scheduleReq->setSrcAddress(selfAddr);
    schedulePacket->addTag<InterfaceReq>()->setInterfaceId(interface80211ptr->getInterfaceId());
    schedulePacket->addTag<PacketProtocolTag>()->setProtocol(&Protocol::manet);
    schedulePacket->addTag<DispatchProtocolReq>()->setProtocol(&Protocol::ipv4);

    send(schedulePacket, "ipOut");
    addToEventLog(selfAddr, Ipv4Address(255, 255, 255, 255), "SCH", "SENT");
}

void Leach::sendDataToCH(Ipv4Address nodeAddr, Ipv4Address CHAddr, double TDMAslot) {
    auto dataPkt = makeShared<LeachDataPkt>();
    dataPkt->setPacketType(DATA);
    double temperature = uniform(0, 1);
    double humidity = uniform(0, 1);
    std::string fingerprint = resolveFingerprint(nodeAddr, getIdealCH(nodeAddr));

    dataPkt->setChunkLength(b(128));
    dataPkt->setTemperature(temperature);
    dataPkt->setHumidity(humidity);
    dataPkt->setSrcAddress(nodeAddr);
    dataPkt->setFingerprint(fingerprint.c_str());
    addToPacketLog(fingerprint);

    auto dataPacket = new Packet("LEACHDataPkt", dataPkt);
    auto addressReq = dataPacket->addTag<L3AddressReq>();
    addressReq->setDestAddress(getIdealCH(nodeAddr));
    addressReq->setSrcAddress(nodeAddr);
    dataPacket->addTag<InterfaceReq>()->setInterfaceId(interface80211ptr->getInterfaceId());
    dataPacket->addTag<PacketProtocolTag>()->setProtocol(&Protocol::manet);
    dataPacket->addTag<DispatchProtocolReq>()->setProtocol(&Protocol::ipv4);

    sendDelayed(dataPacket, TDMAslot, "ipOut");
    addToEventLog(nodeAddr, getIdealCH(nodeAddr), "DATA", "SENT");
    dataPktSent++;
}

void Leach::sendDataToBS(Ipv4Address CHAddr, std::string fingerprint) {
    auto bsPkt = makeShared<LeachBSPkt>();
    bsPkt->setPacketType(BS);
    bsPkt->setChunkLength(b(128));
    bsPkt->setCHAddr(CHAddr);
    bsPkt->setFingerprint(fingerprint.c_str());

    auto bsPacket = new Packet("LEACHBsPkt", bsPkt);
    auto addressReq = bsPacket->addTag<L3AddressReq>();
    addressReq->setDestAddress(Ipv4Address(10, 0, 0, 1));
    addressReq->setSrcAddress(CHAddr);
    bsPacket->addTag<InterfaceReq>()->setInterfaceId(interface80211ptr->getInterfaceId());
    bsPacket->addTag<PacketProtocolTag>()->setProtocol(&Protocol::manet);
    bsPacket->addTag<DispatchProtocolReq>()->setProtocol(&Protocol::ipv4);

    send(bsPacket, "ipOut");
    bsPktSent++;
    setLeachState(nch);
}

Ipv4Address Leach::getIdealCH(Ipv4Address nodeAddr) {
    Ipv4Address tempIdealCHAddr;
    double tempRxPower = -1.0;  // Set to negative to detect if no CH found

    for (auto& it : nodeMemory) {
        if (it.nodeAddr == nodeAddr && it.energy > tempRxPower) {
            tempRxPower = it.energy;
            tempIdealCHAddr = it.CHAddr;
        }
    }

    if (tempRxPower < 0) {
        EV << "Warning: No CH found for node " << nodeAddr.str() << endl;
        // Return a default or broadcast address
        return Ipv4Address(255, 255, 255, 255);
    }

    return tempIdealCHAddr;
}

std::string Leach::resolveFingerprint(Ipv4Address nodeAddr, Ipv4Address CHAddr) {
    std::string CHAddrResolved = std::to_string(CHAddr.getInt());
    std::string nodeAddrResolved = std::to_string(nodeAddr.getInt());
    std::string simTimeResolved = std::to_string(simTime().dbl());

    std::hash<std::string> hashFn;
    size_t hashResolved = hashFn(CHAddrResolved + nodeAddrResolved + simTimeResolved);
    return std::to_string(hashResolved);
}

bool Leach::checkFingerprint(std::string fingerprint) {
    for (auto& it : packetLog) {
        if (it.fingerprint == fingerprint) return true;
    }
    return false;
}

void Leach::addToPacketLog(std::string fingerprint) {
    packetLogEntry packet;
    packet.fingerprint = fingerprint;
    packetLog.push_back(packet);
}

void Leach::addToEventLog(Ipv4Address srcAddr, Ipv4Address destAddr, std::string packet, std::string type) {
    const char* srcNodeName = L3AddressResolver().findHostWithAddress(srcAddr)->getFullName();
    const char* destNodeName;
    if (destAddr.isLimitedBroadcastAddress()) {
        destNodeName = "Broadcast";
    } else {
        cModule* destModule = L3AddressResolver().findHostWithAddress(destAddr);
        if (destModule) {
            destNodeName = destModule->getFullName();
        } else {
            destNodeName = "Unknown";
            EV << "Warning: Could not resolve destination name for " << destAddr.str() << endl;
        }
    }

    J residualCapacity = J(0);
    try {
        SimpleEpEnergyStorage *energyStorageModule = check_and_cast<SimpleEpEnergyStorage*>(host->getSubmodule("energyStorage"));
        residualCapacity = energyStorageModule->getResidualEnergyCapacity();
    } catch (const cRuntimeError& e) {
        EV << "Energy storage error: " << e.what() << endl;
    }

    eventLogEntry nodeEvent;
    nodeEvent.srcNodeName = srcNodeName;
    nodeEvent.destNodeName = destNodeName;
    nodeEvent.time = simTime().dbl() * 1000;
    nodeEvent.packet = packet;
    nodeEvent.type = type;
    nodeEvent.residualCapacity = residualCapacity;
    nodeEvent.state = (leachState == ch ? "ch" : "nch");
    eventLog.push_back(nodeEvent);
}

void Leach::addToNodePosList() {
    IMobility *mobilityModule = check_and_cast<IMobility *>(host->getSubmodule("mobility"));
    Coord pos = mobilityModule->getCurrentPosition();

    nodePositionEntry nodePosition;
    nodePosition.nodeName = host->getFullName();
    nodePosition.posX = pos.getX();
    nodePosition.posY = pos.getY();
    nodePositionList.push_back(nodePosition);
}

void Leach::addToNodeWeightList() {
    nodeWeightObject nodeWeight;
    nodeWeight.nodeName = host->getFullName();
    nodeWeight.weight = weight;
    nodeWeightList.push_back(nodeWeight);
}

void Leach::generateEventLogCSV() {
    std::ofstream eventLogFile("eventLog.csv");
    eventLogFile << "Time,Node,Rx-Tx Node,Packet,Type,Energy,State" << std::endl;
    for (auto& it : eventLog) {
        std::string resolvedResidualCapacity = it.residualCapacity.str().erase(it.residualCapacity.str().size() - 2, 2);
        if (it.type == "SENT") {
            eventLogFile << it.time << "," << it.srcNodeName << "," << it.destNodeName << "," << it.packet << "," << it.type << ","
                         << resolvedResidualCapacity << "," << it.state << std::endl;
        } else {
            eventLogFile << it.time << "," << it.destNodeName << "," << it.srcNodeName << "," << it.packet << "," << it.type << ","
                         << resolvedResidualCapacity << "," << it.state << std::endl;
        }
    }
    eventLogFile.close();
}

void Leach::generateNodePosCSV() {
    std::ofstream nodePosFile("nodePos.csv");
    nodePosFile << "Node,X,Y,weight" << std::endl;
    for (auto& positionIterator : nodePositionList) {
        for (auto& weightIterator : nodeWeightList) {
            if (positionIterator.nodeName == weightIterator.nodeName) {
                nodePosFile << positionIterator.nodeName << "," << positionIterator.posX << "," << positionIterator.posY << ","
                            << weightIterator.weight << std::endl;
                break;
            }
        }
    }
    nodePosFile.close();
}

void Leach::generatePacketLogCSV() {
    std::ofstream packetLogFile("packetLog.csv");
    packetLogFile << "Data-Sent" << std::endl;
    for (auto& packetLogIterator : packetLog) {
        packetLogFile << packetLogIterator.fingerprint << std::endl;
    }
    packetLogFile.close();
}

void Leach::refreshDisplay() const {
    const char *icon;
    switch (leachState) {
        case nch:
            icon = "misc/sensor2";
            break;
        case ch:
            icon = "device/antennatower";
            break;
        default:
            throw cRuntimeError("Unknown LEACH status");
    }
    auto& displayString = getDisplayString();
    displayString.setTagArg("i", 0, icon);
    host->getDisplayString().setTagArg("i", 0, icon);
}

void Leach::finish() {
    addToNodeWeightList();
    generateEventLogCSV();
    generateNodePosCSV();
    generatePacketLogCSV();

    EV << "Total control packets sent by CH: " << controlPktSent << endl;
    EV << "Total control packets received by NCHs from CH: " << controlPktReceived << endl;
    EV << "Total data packets sent to CH: " << dataPktSent << endl;
    EV << "Total data packets received by CH from NCHs: " << dataPktReceived << endl;
    EV << "Total data packets received by CH from NCHs verified: " << dataPktReceivedVerf << endl;
    EV << "Total BS packets sent by CH: " << bsPktSent << endl;

    recordScalar("#dataPktSent", dataPktSent);
    recordScalar("#dataPktReceived", dataPktReceived);
    recordScalar("#dataPktReceivedVerf", dataPktReceivedVerf);
    recordScalar("#controlPktSent", controlPktSent);
    recordScalar("#controlPktReceived", controlPktReceived);
    recordScalar("#bsPktSent", bsPktSent);
    
    // Q-Learning statistics
        if (useQLearning) {
            recordScalar("qTableSize", (int)(qTable.size() / kActions));
            recordScalar("finalExplorationRate", explorationRate);
        
        // Save Q-table if enabled
        if (saveQTable) {
            if (!perNodeQTableFile && host && host->getIndex() != 0) {
                EV << "Skipping Q-table save for non-zero node index (shared table enabled)" << endl;
            } else {
                saveQTableToFile(qTableFilePath);
                EV << "Q-table saved to: " << qTableFilePath << " (size: " << (qTable.size() / kActions) << " states)" << endl;
            }
        }
    }
}

// ========== Q-Learning Implementation ==========

Leach::QLearningState Leach::getCurrentState() {
    QLearningState state;
    
    // Energy level discretization (0-4: very low to high)
    double energyPct = getEnergyPercentage();
    state.energyLevel = discretizeEnergyLevel(energyPct);
    
    // Neighbor count discretization
    state.neighborCount = getNeighborCount();
    
    // Distance to BS (normalized to 0-1, then discretized to 0-3)
    double distance = getDistanceToBS();
    // Normalize distance (assuming max distance ~500m based on network config)
    double normalizedDist = std::min(1.0, distance / 500.0);
    state.distanceBucket = discretizeDistanceBucket(normalizedDist);
    
    // Was CH recently (in last interval)
    int intervalLength = 1.0 / clusterHeadPercentage;
    state.wasCHRecently = (wasCH && fmod(round, intervalLength) < intervalLength / 2) ? 1 : 0;
    
    return state;
}

bool Leach::selectAction(const QLearningState& state) {
    // Epsilon-greedy policy
    double randVal = uniform(0, 1);
    
    if (randVal < explorationRate) {
        // Exploration: random action
        return uniform(0, 1) < 0.5;
    } else {
        // Exploitation: choose action with highest Q-value
        int stateIndex = getStateIndex(state);
        double qNCH = getQValue(stateIndex, 0);
        double qCH = getQValue(stateIndex, 1);
        if (qCH == qNCH) {
            return uniform(0, 1) < 0.5;
        }
        return qCH > qNCH;
    }
}

double Leach::calculateReward() {
    double reward = 0.0;
    
    // Energy-based reward (positive for conserving energy)
    double currentEnergy = getEnergyPercentage();
    double energyChange = currentEnergy - previousEnergy;
    reward += energyChange * 10.0;  // Scale energy reward
    
    // Data transmission reward (positive for successful data delivery)
    int dataPktDelta = (dataPktSent - previousDataPktSent) + (dataPktReceived - previousDataPktReceived);
    reward += dataPktDelta * 5.0;
    
    // Avoid double-penalizing CHs: only penalize if energy is critically low
    if (previousAction && currentEnergy < 0.2) {
        reward -= 2.0;
    }
    
    // Reward for network connectivity (if node has neighbors)
    int neighbors = getNeighborCount();
    reward += neighbors * 0.5;
    
    // Distance-based reward (closer to BS is better for CH)
    if (previousAction) {
        double distance = getDistanceToBS();
        double normalizedDist = std::min(1.0, distance / 500.0);
        reward += (1.0 - normalizedDist) * 3.0;  // Closer = better
    }
    
    return reward;
}

void Leach::updateQValue(const QLearningState& state, bool action, double reward, const QLearningState& nextState, bool nextAction) {
    int stateIndex = getStateIndex(state);
    int nextStateIndex = getStateIndex(nextState);
    int actionIndex = action ? 1 : 0;
    int nextActionIndex = nextAction ? 1 : 0;

    // Get current Q-value
    double currentQ = getQValue(stateIndex, actionIndex);
    
    // Calculate next Q-value based on algorithm
    double nextQ;
    if (learningAlgorithm == "sarsa") {
        // SARSA: Use the actual next action taken (on-policy)
        nextQ = getQValue(nextStateIndex, nextActionIndex);
    } else {
        // Q-learning: Use max Q-value (off-policy)
        nextQ = std::max(getQValue(nextStateIndex, 0), getQValue(nextStateIndex, 1));
    }
    
    // Update rule: Q(s,a) = Q(s,a) + alpha * [reward + gamma * Q(s',a') - Q(s,a)]
    double newQ = currentQ + learningRate * (reward + discountFactor * nextQ - currentQ);
    
    setQValue(stateIndex, actionIndex, newQ);
    
    EV << learningAlgorithm << "-update: state=" << stateToString(state) 
       << ", action=" << (action ? "CH" : "NCH")
       << ", reward=" << reward
       << ", nextQ=" << nextQ
       << ", Q_old=" << currentQ
       << ", Q_new=" << newQ << endl;
}

void Leach::initializeQTable() {
    int totalStates = kEnergyLevels * kNeighborLevels * kDistanceLevels * kWasCHLevels;
    qTable.assign(totalStates * kActions, 0.0);
    if (qTableInitRandom) {
        for (int i = 0; i < totalStates * kActions; ++i) {
            qTable[i] = uniform(qTableInitMin, qTableInitMax);
        }
    }
    EV << "Q-table initialized with " << totalStates << " states" << endl;
}

std::string Leach::stateToString(const QLearningState& state) const {
    std::ostringstream oss;
    oss << "[E:" << state.energyLevel 
        << ",N:" << state.neighborCount
        << ",D:" << state.distanceBucket
        << ",W:" << state.wasCHRecently << "]";
    return oss.str();
}

double Leach::getDistanceToBS() const {
    try {
        IMobility *mobility = check_and_cast<IMobility *>(host->getSubmodule("mobility"));
        Coord nodePos = mobility->getCurrentPosition();
        
        // Base station position (from omnetpp.ini: 290m, 450m)
        Coord bsPos(290.0, 450.0, 0.0);
        
        double distance = nodePos.distance(bsPos);
        return distance;
    } catch (const cRuntimeError& e) {
        EV << "Error getting distance to BS: " << e.what() << endl;
        return 250.0;  // Default distance
    }
}

int Leach::getNeighborCount() const {
    // Count number of CH advertisements from the last completed round
    int count = lastRoundChAdverts;
    
    // Discretize to 0-3
    if (count == 0) return 0;
    else if (count <= 2) return 1;
    else if (count <= 4) return 2;
    else return 3;
}

int Leach::discretizeEnergyLevel(double energyPercentage) const {
    // Discretize energy to 5 levels (0-4)
    if (energyPercentage < 0.2) return 0;      // Very low
    else if (energyPercentage < 0.4) return 1; // Low
    else if (energyPercentage < 0.6) return 2; // Medium
    else if (energyPercentage < 0.8) return 3; // High
    else return 4;                              // Very high
}

void Leach::saveQTableToFile(const std::string& filepath) {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        EV << "Warning: Could not open file for writing: " << filepath << endl;
        return;
    }
    
    file << "# qtable_v1\n";
    file << "# algorithm=" << learningAlgorithm << "\n";
    file << "# explorationRate=" << explorationRate << "\n";
    file << "# dimensions=" << kEnergyLevels << "," << kNeighborLevels << ","
         << kDistanceLevels << "," << kWasCHLevels << "," << kActions << "\n";
    file << "# format: stateIndex,q_nch,q_ch\n";

    int totalStates = kEnergyLevels * kNeighborLevels * kDistanceLevels * kWasCHLevels;
    for (int i = 0; i < totalStates; ++i) {
        double qNch = getQValue(i, 0);
        double qCh = getQValue(i, 1);
        file << i << "," << qNch << "," << qCh << "\n";
    }
    file.close();
    
    EV << "Q-table saved successfully to " << filepath << " (" << (qTable.size() / kActions) << " states)" << endl;
}

void Leach::loadQTableFromFile(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        EV << "No existing Q-table found at " << filepath << ", starting with empty table" << endl;
        return;
    }
    
    std::string line;
    int loadedStates = 0;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            if (line.find("# explorationRate=") == 0) {
                std::string valueStr = line.substr(std::string("# explorationRate=").size());
                try {
                    double loadedEpsilon = std::stod(valueStr);
                    explorationRate = std::max(minExplorationRate, loadedEpsilon);
                } catch (...) {
                    EV << "Warning: Failed to parse explorationRate from Q-table header" << endl;
                }
            }
            continue;
        }
        std::istringstream iss(line);
        std::string token;
        int index = -1;
        double qNch = 0.0;
        double qCh = 0.0;

        if (std::getline(iss, token, ',')) {
            index = std::stoi(token);
        }
        if (std::getline(iss, token, ',')) {
            qNch = std::stod(token);
        }
        if (std::getline(iss, token, ',')) {
            qCh = std::stod(token);
        }

        int totalStates = kEnergyLevels * kNeighborLevels * kDistanceLevels * kWasCHLevels;
        if (index >= 0 && index < totalStates) {
            setQValue(index, 0, qNch);
            setQValue(index, 1, qCh);
            loadedStates++;
        }
    }
    file.close();

    EV << "Loaded Q-table from " << filepath << " (" << loadedStates << " states)" << endl;
}

int Leach::discretizeDistanceBucket(double normalizedDistance) const {
    if (normalizedDistance <= 0.25) return 0;
    else if (normalizedDistance <= 0.5) return 1;
    else if (normalizedDistance <= 0.75) return 2;
    return 3;
}

int Leach::getStateIndex(const QLearningState& state) const {
    int e = std::max(0, std::min(kEnergyLevels - 1, state.energyLevel));
    int n = std::max(0, std::min(kNeighborLevels - 1, state.neighborCount));
    int d = std::max(0, std::min(kDistanceLevels - 1, state.distanceBucket));
    int w = std::max(0, std::min(kWasCHLevels - 1, state.wasCHRecently));

    int idx = (((e * kNeighborLevels) + n) * kDistanceLevels + d) * kWasCHLevels + w;
    return idx;
}

double Leach::getQValue(int stateIndex, int action) const {
    int totalStates = kEnergyLevels * kNeighborLevels * kDistanceLevels * kWasCHLevels;
    int idx = stateIndex * kActions + action;
    if (stateIndex < 0 || stateIndex >= totalStates || action < 0 || action >= kActions) {
        return 0.0;
    }
    if (idx < 0 || idx >= (int)qTable.size()) {
        return 0.0;
    }
    return qTable[idx];
}

void Leach::setQValue(int stateIndex, int action, double value) {
    int totalStates = kEnergyLevels * kNeighborLevels * kDistanceLevels * kWasCHLevels;
    int idx = stateIndex * kActions + action;
    if (stateIndex < 0 || stateIndex >= totalStates || action < 0 || action >= kActions) {
        return;
    }
    if (idx < 0 || idx >= (int)qTable.size()) {
        return;
    }
    qTable[idx] = value;
}

std::string Leach::resolveQTableFilePath(const std::string& basePath) const {
    if (!perNodeQTableFile) {
        return basePath;
    }
    int nodeIndex = 0;
    if (host) {
        nodeIndex = host->getIndex();
    }

    std::string suffix = "_node" + std::to_string(nodeIndex);
    size_t lastSlash = basePath.find_last_of("/\\");
    size_t lastDot = basePath.find_last_of('.');
    if (lastDot != std::string::npos && (lastSlash == std::string::npos || lastDot > lastSlash)) {
        return basePath.substr(0, lastDot) + suffix + basePath.substr(lastDot);
    }
    return basePath + suffix;
}

J Leach::getNodeCurrentEnergy() const {
    try {
        SimpleEpEnergyStorage *energyStorageModule = check_and_cast<SimpleEpEnergyStorage*>(host->getSubmodule("energyStorage"));
        return energyStorageModule->getResidualEnergyCapacity();
    } catch (const cRuntimeError& e) {
        EV << "Energy storage error: " << e.what() << endl;
        return J(0);
    }
}

double Leach::getEnergyPercentage() const {
    try {
        SimpleEpEnergyStorage *energyStorageModule = check_and_cast<SimpleEpEnergyStorage*>(host->getSubmodule("energyStorage"));
        J residual = energyStorageModule->getResidualEnergyCapacity();
        J nominal = energyStorageModule->getNominalEnergyCapacity();
        
        if (nominal.get() > 0) {
            return residual.get() / nominal.get();
        }
        return 0.0;
    } catch (const cRuntimeError& e) {
        EV << "Energy storage error: " << e.what() << endl;
        return 0.0;
    }
}

} // namespace inet
