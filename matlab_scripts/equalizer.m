### This script performs zero-forcing equalization at the input data.
### The input data is assumed to be synchronized to around the actual
### start of the first data symbol


function equalizer(samples_chann)

pkg load communications
ofdm_symbols = 16;               % Number of ofdm symbols present at the input data

nfft = 1024;                    % nfft size equal to number of subcarriers
BW = 368.64e6;                % Bandwidth of subcarriers (-F/2 F/2)
M = 128;                        % Baseband modulation order
cp_len = nfft/4;                % Cyclic Prefix length

rep = 4;                        %   Repetition of pilots
pilot_index = [112 : rep :  511, 514 : rep : 913];
data_index = 1:nfft;            % Index of data subcarriers
snr = 30;                       % Signal to noise ratio
zp_index = [1:111,512,513,914:1024];
pilot_carriers = length(pilot_index);
zp_carriers = length(zp_index);
data_index([zp_index,pilot_index]) = [];
data_carriers = length(data_index);
scs = BW/nfft;
plot_v = 0;
tx_scale = 1.1e1;
scale = 2.6e3;
Ts = 1/(BW);                      % Sample period
t = 0:Ts:(nfft+cp_len-1)*Ts*100;
P = text2octave("pilots.txt",2^15);
P = reshape(P, [200 , 1]);

# The number of repetitions of P must be equal to the number of symbols contained in the input data
P = [P ,P, P,P ,P, P,P ,P,P ,P, P,P ,P, P,P ,P];
ofdm_signal_rx1 = samples_chann(1 : (ofdm_symbols) * 1280);
ofdm_signal_par = reshape(ofdm_signal_rx1,[nfft+cp_len ofdm_symbols]);
cp_rm = ofdm_signal_par;
cp_rm(1:cp_len,:) = [];
ofdm_cfo_signal_par_eq = [];
cfo_est = 0;
tanvals = [];
sumvals =[];
s_r = 0;
s_i = 0;
sum_re = [];
sum_im = [];
for p = 1:ofdm_symbols
    cp_cfo_begin = ofdm_signal_par(1:cp_len,p);             % Obtain cyclic prefix at beginning of symbol
    cp_cfo_end = (ofdm_signal_par(nfft+1:end,p));       % Obtain cyclic prefix at end of symbol
    %cp_cfo_est = filter(cp_cfo_end,1,cp_cfo_begin);             % Correlate
    tmp = cp_cfo_end.*conj(cp_cfo_begin);
    mult_val = complex(real(cp_cfo_end.*conj(cp_cfo_begin)),imag(cp_cfo_end.*conj(cp_cfo_begin)));
    for k = 1: length(mult_val)
    s_r = s_r + real(mult_val(k));
    s_i = s_i + imag(mult_val(k));
    sum_re = [sum_re s_r];
    sum_im =  [sum_im s_i];
    endfor
    sum_val = sum(mult_val);
    tan_val = atan2(imag(sum_val),real(sum_val));
    tanvals = [tanvals tan_val];
    normalized_cfo_est = 1*tan_val;
    cfo_est = normalized_cfo_est * scs;
    sinus = cos(cfo_est*t(1:nfft))+1i*sin(cfo_est*t(1:nfft));
    #if(p < 8)
      #   ofdm_cfo_signal_par_eq(:,p) = cp_rm(:,p).*inter(1 + 1024 * (p-1) : p * 1024)';
     #else
        ofdm_cfo_signal_par_eq(:,p) = cp_rm(:,p).*(cos(cfo_est*t(1:nfft))+1i*sin(cfo_est*t(1:nfft)))';
     # endif
end

fft_out_cfo = fft(ofdm_cfo_signal_par_eq,nfft);
fft_out_cfo(:,1) = fftshift(fft_out_cfo(:,1));
fft_out_cfo(:,2) = fftshift(fft_out_cfo(:,2));
fft_out_cfo(:,3) = fftshift(fft_out_cfo(:,3));
fft_out_cfo(:,4) = fftshift(fft_out_cfo(:,4));
fft_out_cfo(:,5) = fftshift(fft_out_cfo(:,5));
fft_out_cfo(:,6) = fftshift(fft_out_cfo(:,6));
fft_out_cfo(:,7) = fftshift(fft_out_cfo(:,7));
fft_out_cfo(:,8) = fftshift(fft_out_cfo(:,8));
fft_out_cfo(:,9) = fftshift(fft_out_cfo(:,9));
fft_out_cfo(:,10) = fftshift(fft_out_cfo(:,10));
fft_out_cfo(:,11) = fftshift(fft_out_cfo(:,11));
fft_out_cfo(:,12) = fftshift(fft_out_cfo(:,12));
fft_out_cfo(:,13) = fftshift(fft_out_cfo(:,13));
fft_out_cfo(:,14) = fftshift(fft_out_cfo(:,14));
fft_out_cfo(:,15) = fftshift(fft_out_cfo(:,15));
fft_out_cfo(:,16) = fftshift(fft_out_cfo(:,16));

P_RX = fft_out_cfo(pilot_index,:);
num = conj(P_RX); denom = conj(P);
num_abs = abs(num); num_ang = angle(num); denom_abs = abs(denom); denom_ang = angle(denom) * -1;
H_ang = num_ang - denom_ang;
H_abs = num_abs ./ denom_abs;
H = H_abs .* exp(1i*H_ang);
%H = num ./ denom;
H_ang1 = H_ang;
H_abs1 = H_abs;


% Normal ZF equalization
H_ZF_interp = interp1(pilot_index,H,1:nfft,'nearest');
Z_EQ_ZF = fft_out_cfo ./ conj(H_ZF_interp);

H_ZF_interp_abs = interp1(pilot_index,H_abs1,1:nfft,'previous');
H_ZF_interp_ang = interp1(pilot_index,H_ang1,1:nfft,'previous');

for i = 1 : ofdm_symbols
H_ZF_interp_abs(911,i) = H_ZF_interp_abs(910,i) ;
H_ZF_interp_abs(912,i) = H_ZF_interp_abs(910,i) ;
H_ZF_interp_abs(913,i) = H_ZF_interp_abs(910,i) ;
H_ZF_interp_ang(911,i) = H_ZF_interp_ang(910,i) ;
H_ZF_interp_ang(912,i) = H_ZF_interp_ang(910,i) ;
H_ZF_interp_ang(913,i) = H_ZF_interp_ang(910,i) ;
endfor
Z_EQ_ZF_abs = abs(fft_out_cfo) ./ H_ZF_interp_abs;
Z_EQ_ZF_abs(zp_index,:) = [];

Z_EQ_ZF_ang = angle(fft_out_cfo) + H_ZF_interp_ang;
Z_EQ_ZF_ang(zp_index,:) = [];

Z_EQ_ZF1 = Z_EQ_ZF_abs .* exp(1j*Z_EQ_ZF_ang);
Z_EQ_ZF1 = reshape(Z_EQ_ZF1, [1, ofdm_symbols*800]);
pilot_index = 1:4:length(Z_EQ_ZF1);
Z_EQ_ZF1(pilot_index) = [];

# The Z_EQ_ZF1 array contains the equalized symbols
scatterplot(Z_EQ_ZF1 );


xlim([-2 2]);
ylim([-1 1]);
txed = [];
for i = 1 : length(Z_EQ_ZF1)
        if(real(Z_EQ_ZF1))
                txed = [txed complex(0.5,0)];
         else
                txed = [txed complex(-0.5,0)];
         end
endfor
error_vector = Z_EQ_ZF1 - txed;
EVM1 = sqrt(mean(abs(error_vector).^2)) / sqrt(mean(abs(txed).^2)) * 100; % EVM in percentage
EVM1 = sqrt(real(error_vector).^2 + imag(error_vector).^2 ) / abs(txed)
fprintf('EVM: %.2f%%\n', EVM1);

end
