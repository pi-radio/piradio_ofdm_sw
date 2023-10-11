# This script demodulates the input symbols. It takes as input the output of the equalizer script.
i = 1;
f2=Z_EQ_ZF1;
cnt = 1;
samp = 0;
index = 1;
idx_bits = 1;
output = zeros(1,180);
bits = zeros(1,800);
for i = 1 : 1024
        if( (i != 1) && (i != 1024) && ((i <= 401 && (mod( i - 2, 5) != 0)) || (i >= 624 && mod(i - 4, 5)!= 0)) )
                if(cnt <= 32)
                        if(real(f2(i)) > 0)
                                samp = bitset(samp, cnt, 1);
                                bits(idx_bits++) = 1;
                        else
                                samp = bitset(samp, cnt, 0);
                                bits(idx_bits++) = 0;
                        endif
                        cnt++;
                else
                        cnt = 1;
                        output(index++) = samp;
                       if(real(f2(i)) > 0)
                                samp = bitset(samp, cnt, 1);
                                bits(idx_bits++) = 1;
                        else
                                samp = bitset(samp, cnt, 0);
                                bits(idx_bits++) = 0;
                        endif
                        cnt++;
                endif
        endif
endfor
