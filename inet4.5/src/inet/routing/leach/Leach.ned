
// following repository is used for the LEACH protocol:
//  https://github.com/xcodeBn

package inet.routing.leach;

import inet.routing.contract.IManetRouting;

simple Leach like IManetRouting
{
    parameters:
        @display("i=block/routing");
        string interfaceTableModule;   // The path to the InterfaceTable module
        string routingTableModule;
        double helloInterval @unit(s) = default(5s);  // how often hello messages should be sent out
        string netmask = default("255.255.0.0");  // netmask for the ad-hoc network
        double maxVariance = default(1); // This is the maximum of a random value to determine when the first hello message will be sent out
        double routeLifetime @unit(s) = default(5s);  // the expiry time of the routing entries
        bool manetPurgeRoutingTables = default(true);
        volatile double broadcastDelay @unit(s) = default(uniform(0s,0.01s));
        
        double dataPktSendDelay @unit(s) = default(7s);
        double CHPktSendDelay @unit(s) = default(15s);
        double roundDuration @unit(s) = default(30s);
        
        int numNodes = default(10);
        double clusterHeadPercentage = default(0.5);
        
        // Q-Learning parameters
        bool useQLearning = default(true);  // Enable Q-learning based CH election
        string learningAlgorithm = default("qlearning");  // "qlearning" or "sarsa"
        double qLearningAlpha = default(0.1);  // Learning rate (alpha)
        double qLearningGamma = default(0.9);  // Discount factor (gamma)
        double qLearningEpsilon = default(0.3);  // Initial exploration rate
        double qLearningEpsilonDecay = default(0.995);  // Exploration decay rate per round
        double qLearningMinEpsilon = default(0.01);  // Minimum exploration rate
        bool qTableInitRandom = default(true);  // Randomize initial Q-table values
        double qTableInitMin = default(-0.01);
        double qTableInitMax = default(0.01);
        
        // Q-table persistence
        bool saveQTable = default(true);  // Save Q-table at end of simulation
        bool loadQTable = default(true);  // Load Q-table at start of simulation
        string qTableFilePath = default("qtable.csv");  // File path for Q-table
        bool perNodeQTableFile = default(true);  // Add node index to Q-table file name
    gates:
        input ipIn;
        output ipOut;
}
