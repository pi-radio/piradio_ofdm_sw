### This function reads from a file containing 32bit interleaved I/Q values in hex notation
### and converts it to matlab-compatible samples for processing

### filename : the filename containing the 32bit values
### scale: the scaling to be applied to each I and Q sample. e.g. for 16 bit samples, 32768

function samples = text2octave(filename, scale)
fid = fopen(filename, 'r');
[x, count] = fscanf(fid, '%lx');
samples = [];
q_signed = 0;
i_signed = 0;
bits = 16
neg = 0;
s = 0;
for i = 1 : length(x)
      q = x(i) / (2^bits);
      neg = bitget(q, bits);
      q = int32(bitand(q, 0x0000ffff));
      if(neg == 1)
          q = bitset(q,bits,1);
      else
          q = bitset(q,bits,0);
      endif
      if(neg)
          q_signed = -1 * (2 * (2^(bits -1) - 1) + 2 - q);
      else
          q_signed = q;
      endif

      re = x(i);
      re = int32(bitand(re, 0x0000ffff));
      if(bitget(x(i), bits) == 1)
          re = bitset(re,bits,1);
      else
          re = bitset(re,bits,0);
      endif
      if(bitget(x(i), bits) == 1)
          i_signed = -1 * (2 * (2^(bits -1) - 1) + 2 - re);
      else
          i_signed = re;
      endif
    samples = [samples complex(double(i_signed)/scale ,double( q_signed) / scale)];
endfor
end

