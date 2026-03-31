@echo off
cd /d "C:\Users\Usuario\AppData\Local\Temp\codex_heur_warmstarts\heuristico"
renault.exe -Iter 100 -InputFile "\\wsl.localhost\archlinux\home\juan\Documents\CODE Julia\results\heur_warmstart_rand_iter100\BY2_Camiones135\\"
exit /b %ERRORLEVEL%
