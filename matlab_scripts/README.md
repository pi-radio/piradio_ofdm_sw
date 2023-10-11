## Piradio OFDM Matlab/Octave scripts

These scripts implement the algorithms of the PiRadio OFDM system. They are intended to use the output samples as captured by ILA cores, and stored as text files, but with proper modification of the way the data is read, also binary input files can be used.

The tx.txt file serves as an example of input data and contains a full piradio OFDM transmit frame as captured by an ILA core. A sample execution can be the following:

samples = text2octave("tx.txt",32768); // Read the input samples
correlate_ofdm_tx(samples); // See the output of the correlation with the synchronization word
equalizer(samples(1281 : end));  //Get the equalized samples. The input to the script assumes that it is synchronized to the start of the first symbol, hence we give as parameter the vector of samples starting right after the sync word symbol.

A python script for converting files containing 128bit text hex values is provided. It takes as arguments the input filename and output filename:

python breakup128to32.py input128.txt input32.txt

