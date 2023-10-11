### This script generates a piradio TX time domain signal

# number of symbols to produce
num_symbols = 17;

P = text2octave("pilots.txt",2^15);
P = reshape(P, [200 , 1]);
P = 2*P;
P = [P ;1];
time_domain = [];
pilot_idx = 1;
td = [];
for i = 1 : num_symbols
        sym = zeros(1,111);
        pilot_idx = 1;
        for j = 1 : 400
          if(mod(j - 1, 4) == 0)
                sym = [sym P(pilot_idx + 1)];
                pilot_idx = pilot_idx + 1;
          else
                 if (randi([0 1]))
                        sym = [sym 1];
                 else
                        sym = [sym -1];
                 endif
         endif
        endfor
        sym = [sym zeros(1,2)];
         for j = 1 : 400
         if(mod(j - 1, 4) == 0)
                sym = [sym P(pilot_idx + 1)];
                pilot_idx = pilot_idx + 1;
          else
                 if (randi([0 1]))
                        sym = [sym 1];
                 else
                        sym = [sym -1];
                 endif
           endif
        endfor
        sym = [sym zeros(1,111)];
        t_sim = ifft(ifftshift(sym));
        t_sim = [t_sim(769 : end) t_sim];
        time_domain = [time_domain t_sim];

        tt = ifft(ifftshift(samples( (i - 1) * 1024 + 1 : i * 1024)));
        tts =  [tt(769 : end) tt];
        td = [td tts];
endfor

