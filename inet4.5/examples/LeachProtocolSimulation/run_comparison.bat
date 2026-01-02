@echo off
REM Batch script to run multiple simulations for comparison
REM Run this from the Simulation/LeachProtocolSimulation directory

echo ========================================
echo LEACH Protocol Comparison Script
echo ========================================
echo.

set NUM_RUNS=10

echo Running Original LEACH (Probabilistic)...
echo ----------------------------------------
for /L %%i in (1,1,%NUM_RUNS%) do (
    echo [%date% %time%] Run %%i of %NUM_RUNS%
    ../../inet -c LEACHPROTOCOL -f omnetpp_original.ini -n . -r %%i
    if errorlevel 1 (
        echo ERROR: Simulation %%i failed!
        pause
        exit /b 1
    )
)

echo.
echo Running Q-Learning LEACH...
echo ----------------------------------------
for /L %%i in (1,1,%NUM_RUNS%) do (
    echo [%date% %time%] Run %%i of %NUM_RUNS%
    ../../inet -c LEACHPROTOCOL -f omnetpp_qlearning.ini -n . -r %%i
    if errorlevel 1 (
        echo ERROR: Simulation %%i failed!
        pause
        exit /b 1
    )
)

echo.
echo ========================================
echo All simulations completed successfully!
echo ========================================
echo.
echo Results are stored in:
echo   - results/LEACHPROTOCOL-* (Original LEACH)
echo   - results/LEACHPROTOCOL-* (Q-Learning LEACH)
echo.
echo Use OMNeT++ Analysis Tool to compare results.
echo.
pause

