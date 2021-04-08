SET PATH=D:\Program\ARM\ARMCC\Bin;C:\Program Files (x86)\Common Files\Oracle\Java\javapath;C:\WINDOWS\system32;C:\WINDOWS;C:\WINDOWS\System32\Wbem;C:\WINDOWS\System32\WindowsPowerShell\v1.0\;C:\WINDOWS\System32\OpenSSH\;D:\Program\Git\cmd;C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility;C:\Program Files\Intel\WiFi\bin\;C:\Program Files\Common Files\Intel\WirelessCommon\;D:\Program\MATLAB\R2019b\bin;D:\Program\NI\Shared\LabVIEW CLI;C:\Program Files (x86)\IVI Foundation\VISA\WinNT\Bin\;C:\Program Files\IVI Foundation\VISA\Win64\Bin\;C:\Program Files (x86)\IVI Foundation\VISA\WinNT\Bin;C:\Users\Laptop_luyi\AppData\Local\Microsoft\WindowsApps;C:\Users\Laptop_luyi\AppData\Local\GitHubDesktop\bin;C:\Users\Laptop_luyi\AppData\Local\Microsoft\WindowsApps;;D:\Program\Microsoft VS Code\bin;D:\altera\13.0\modelsim_ase\win32aloem
SET CPU_TYPE=STM32F407IGHx
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=Flash
SET CPU_CLOCK=0x00B71B00
"D:\Program\ARM\ARMCC\Bin\ArmCC" --Via "..\build\mdk-arm\init.__i"
"D:\Program\ARM\ARMCC\Bin\ArmCC" --Via "..\build\mdk-arm\app_manage.__i"
"D:\Program\ARM\ARMCC\Bin\ArmCC" --Via "..\build\mdk-arm\communicate.__i"
"D:\Program\ARM\ARMCC\Bin\ArmCC" --Via "..\build\mdk-arm\offline_service.__i"
"D:\Program\ARM\ARMCC\Bin\ArmCC" --Via "..\build\mdk-arm\sensor_task.__i"
"D:\Program\ARM\ARMCC\Bin\ArmCC" --Via "..\build\mdk-arm\shell.__i"
"D:\Program\ARM\ARMCC\Bin\ArmCC" --Via "..\build\mdk-arm\chassis_app.__i"
"D:\Program\ARM\ARMCC\Bin\ArmLink" --Via "..\build\MDK-ARM\standard.lnp"
"D:\Program\ARM\ARMCC\Bin\fromelf.exe" "..\build\MDK-ARM\standard.axf" --i32combined --output "..\build\MDK-ARM\standard.hex"
fromelf.exe  --bin -o ./bin/standard.bin ../build/MDK-ARM/standard.axf
fromelf.exe  -c ../build/MDK-ARM/standard.axf -o ./bin/standard.lss
