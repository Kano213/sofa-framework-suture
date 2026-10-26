@echo off
chcp 65001 > nul
title Simulation Chirurgicale de Suture - Version Finale (SOFA)
echo ============================================================================
echo   LANCEMENT DE LA SIMULATION CHIRURGICALE DE SUTURE (VERSION FINALE)
echo ============================================================================
"C:\Users\djaou\SOFA\v26.06.00\bin\runSofa.exe" "%~dp0skin_simulation.py"
pause
