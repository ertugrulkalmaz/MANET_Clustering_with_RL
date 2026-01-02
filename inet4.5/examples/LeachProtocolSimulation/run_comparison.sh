#!/bin/bash
# Shell script to run multiple simulations for comparison
# Run this from the Simulation/LeachProtocolSimulation directory

echo "========================================"
echo "LEACH Protocol Comparison Script"
echo "========================================"
echo ""

NUM_RUNS=10

echo "Running Original LEACH (Probabilistic)..."
echo "----------------------------------------"
for i in $(seq 1 $NUM_RUNS); do
    echo "[$(date)] Run $i of $NUM_RUNS"
    ../../inet -c LEACHPROTOCOL -f omnetpp_original.ini -n . -r $i
    if [ $? -ne 0 ]; then
        echo "ERROR: Simulation $i failed!"
        exit 1
    fi
done

echo ""
echo "Running Q-Learning LEACH..."
echo "----------------------------------------"
for i in $(seq 1 $NUM_RUNS); do
    echo "[$(date)] Run $i of $NUM_RUNS"
    ../../inet -c LEACHPROTOCOL -f omnetpp_qlearning.ini -n . -r $i
    if [ $? -ne 0 ]; then
        echo "ERROR: Simulation $i failed!"
        exit 1
    fi
done

echo ""
echo "========================================"
echo "All simulations completed successfully!"
echo "========================================"
echo ""
echo "Results are stored in:"
echo "  - results/LEACHPROTOCOL-* (Original LEACH)"
echo "  - results/LEACHPROTOCOL-* (Q-Learning LEACH)"
echo ""
echo "Use OMNeT++ Analysis Tool to compare results."
echo ""

