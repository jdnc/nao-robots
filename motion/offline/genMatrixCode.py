#!/usr/bin/env python

import scipy.io
import subprocess, sys

argList = [['MEMORY_ROBOT',0.01,100],['MEMORY_SIM',0.02,100]]

heights = range(200,400,5)

f = open('../initWalkMatrices.cpp','w')
f.write('#include "WalkModule.h"\n\n')
f.write('// GENERATED via %s\n' % sys.argv[0])
f.write('void WalkModule::initMatrices(int height) {\n')

for height in heights:
  f.write('if (height == %i) {\n' % height)
  for i,args in enumerate(argList):
    p = subprocess.Popen(['octave','setupobserver.m'],stdout=subprocess.PIPE,stdin=subprocess.PIPE)
    inp = '%f\n%i\n%i\n' % (args[1],args[2],height)
    out,err = p.communicate(inp)

    res = scipy.io.loadmat('results.mat')
    
    if (i != 0):
      f.write('else ')
    f.write('if (frame_info_->source == %s) {\n' % args[0])
    f.write('float gd[] = {%s};\n\n' % ','.join(map(str,res['Gd'][0])))
    f.write('num_preview_frames_ = %i;\n' % len(res['Gd'][0]))
    f.write('if (num_preview_frames_ > MAX_PREVIEW_FRAMES) {\n')
    f.write('std::cerr << "num_preview_frames: " << num_preview_frames_ << " greater than MAX_PREVIEW_FRAMES: " << MAX_PREVIEW_FRAMES << endl;\n }\n')
    f.write('for (unsigned int i = 0; i < num_preview_frames_ && i < MAX_PREVIEW_FRAMES; i++)\n  Gd_[i] = gd[i];')
    f.write('Gi_ = %f;\n' % res['Gi'][0])
    
    f.write('\n')
    for i in range(3):
      f.write('Gx_[0][%i] = %f;\n' % (i,res['Gx'][0][i]))
    
    for i in range(3):
      f.write('\n')
      for j in range(3):
        f.write('A0_[%i][%i] = %f;\n' % (i,j,res['A0'][i][j]))

    f.write('\n')
    for i in range(3):
      f.write('b0_[%i][0] = %f;\n' % (i,res['b0'][i][0]))
    
    f.write('\n')
    for i in range(3):
      f.write('L_[%i][0] = %f;\n' % (i,res['L'][i][0]))
      f.write('L_[%i][1] = %f;\n' % (i,res['L'][i][1]))

    f.write('\n')
    for i in range(3):
      f.write('c0_[0][%i] = %f;\n' % (i,res['c0'][0][i]))

    f.write('}\n')
  f.write('}\n')

f.write('\n}\n')
