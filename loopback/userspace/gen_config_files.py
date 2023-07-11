import sys
import random
import numpy
import struct
import ctypes

sync_word= [0x6b,0x3c,0x87,0xf2,0x30,0x8d,0x00,0x20,0x7d,0xbf,0x1b,0x89,0x42,0x97,0xec,0x79,0x0f,0x27,0x41,0x3e,0x5e,0x17,0x9c,0x8d,0x7c,0x76,0xbe,0x26,0x07,0x39,0x46,0xcf,0x54,0x47,0x5d,0x7a,0x28,0x12,0xe8,0x2b,0x06,0x28,0x25,0x13,0x09,0x2d,0x4b,0x8b,0x14,0x0b,0xf8,0x75,0x15,0xbf,0xdb,0x85,0x62,0xc8,0xf6,0x0f,0x79,0x0f,0xc2,0xb7,0x34,0x24,0x1c,0x66,0x57,0x13,0x04,0x64,0x2f,0xee,0xc4,0x05,0x4a,0x66,0x1b,0x7f,0x47,0xbc,0xab,0xbe,0x7e,0x7b,0xfc,0xc4,0x76,0xc4,0x0a,0xb4,0x38,0x1d,0xc2,0x71,0x0a,0xfa,0x2c,0x4b];

sync_word_little = [];

for i in range(len(sync_word)):
        sync_word_little.extend([sync_word[(i//4)*4 + (3 - (i % 4))]]);
modulated = []
index = 0
pilots_every = 4;
template = [];
total_zeros = 0;
pilots = [];
map = numpy.zeros((32,), dtype=int);
for i in range(1024):
    if(i < 111 or i > 912 or i == 511 or i == 512):
    #if((i == 0 or i == 1023) or (i>400 and i < 623)):
        modulated.append((0))
        total_zeros = total_zeros + 1;
        print(total_zeros)
    else:
        if((sync_word_little[index//8] >> (index % 8)) & 1 == 0):
           modulated.append((0xc000))
        else:
           modulated.append((0x3fff))
        index = index + 1;

index = 0;
for i in range(1024):
    #if((i>400 and i < 623) or (i == 0) or i == 1023):
    if(i < 111 or i > 912 or i == 511 or i == 512): 
        template.append((0))
    else:
        if(index % pilots_every == 0):
            if(random.randint(0,1)):
                template.append((0x3fff))
                pilots.append((0x3fff))
            else:
                template.append((0xc000))
                pilots.append((0xc000))
        else:
            template.append((0))
            map[i//32] = map[i//32] | (1 << i % 32);
        #if((sync_word_little[index//8] >> (index % 8)) & 1 == 0):
        #    modulated.append((0xc000))
        #else:
        #    modulated.append((0x3fff))
        index = index + 1;
p =  2;
print(total_zeros)
with open('sync_temp.txt', 'w') as f:
    for i in modulated:
        hex_str = hex(i)[2:].zfill(8)
        f.write(hex_str + '\n')
    for i in template:
        hex_str = hex(i)[2:].zfill(8)
        f.write(hex_str + '\n')
with open('pilots.txt', 'w') as f:
    for i in pilots:
        hex_str = hex(i)[2:].zfill(8)
        f.write(hex_str + ',\n')
with open('map.txt', 'w') as f:
    for i in map:
        hex_str = hex(i)[2:].zfill(8)
        f.write(hex_str + '\n')
with open('sync_temp.bin', 'wb') as f:
    binary_data = struct.pack('<{}i'.format(len(modulated)), *modulated)
    f.write(binary_data)
    binary_data = struct.pack('<{}i'.format(len(template)), *template)
    f.write(binary_data)
with open('map.bin', 'wb') as f:
    binary_data = struct.pack('<{}I'.format(len(map)), *map)
    f.write(binary_data)
with open('sync_word.bin', 'wb') as f:
    binary_data = struct.pack('<{}I'.format(len(modulated)), *modulated)
    f.write(binary_data)
