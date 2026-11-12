import {spawnSync} from 'node:child_process';import {mkdirSync} from 'node:fs';import {join} from 'node:path';
mkdirSync('build',{recursive:true});const exe=process.platform==='win32'?'wro-sim-engine.exe':'wro-sim-engine';
const original='original/WRO2026Senior-new_drivebase_shuga/main/src/';const args=['-std=c++20','-O2','-pthread','-Isimulator/native/include','-Ioriginal/WRO2026Senior-new_drivebase_shuga/main/include','simulator/native/main.cpp','simulator/native/virtual_arduino.cpp',...['motion_lib.cpp','servo_lib.cpp','pi_color_lib.cpp','move_function.cpp','extra.cpp','main.cpp'].map(x=>original+x),'-o',join('build',exe)];
const r=spawnSync(process.env.CXX||'g++',args,{stdio:'inherit'});process.exit(r.status??1);
