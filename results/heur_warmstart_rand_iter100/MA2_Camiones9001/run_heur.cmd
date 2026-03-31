@echo off
cd /d "C:\Users\Usuario\AppData\Local\Temp\codex_heur_warmstarts\heuristico"
renault.exe -Iter 100 -InputFile "\\wsl.localhost\archlinux\home\juan\Documents\CODE Julia\results\heur_warmstart_rand_iter100\MA2_Camiones9001\\"
exit /b %ERRORLEVEL%
