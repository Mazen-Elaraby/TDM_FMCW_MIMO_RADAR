TDM FMCW MIMO Radar Signal Processing Chain 

This project is an implementation of the basic signal processing chain of an ADAS Automotive TDM FMCW MIMO Radar System. An increased attention is given to the concept of virtual arrays and their role in increasing angular resolution.

## Radar System Setup

The Radar system modelled is a Uniform Linear Array (ULA) with the following characteristics :
- Transmitter:
	- Peak Power: 0.001 Watts
	- Gain : 36 dB
- Receiver Preamp
	- Gain: 40 dB
	- Noise Figure: 4.5 dB

The Radar system is configured as follows:

- Operating frequency (GHz):    77
- Maximum target range (m):     200
- Range resolution (m):         1
- Maximum target speed (km/h):  230
- Sweep time (microseconds):    7.33
- Sweep bandwidth (MHz):       150
- Maximum beat frequency: (MHz) 27.30

Note:
Only main components are modeled and the effects from other components are omitted, such as coupler and mixer. In addition, the antenna is assumed to be isotropic.

## FMCW Waveform

An up-sweep linear FMCW signal

![Screenshot 2023-05-25 013929](https://github.com/Mazen-Elaraby/TDM_FMCW_MIMO_RADAR/assets/99294980/dc19f27a-5170-4edd-b408-10590a17a781)

## TDM FMCW MIMO & Virtual Arrays

In a coherent MIMO radar system, each antenna of the transmit array transmits an orthogonal waveform. Because of this orthogonality, it is possible to recover the transmitted signals at the receive array. The measurements at the physical receive array corresponding to each orthogonal waveform can then be stacked to form the measurements of the virtual array.

The following diagram shows the equivalence of the two-way pattern of the physical array with the virtual one:

![untitled1](https://github.com/Mazen-Elaraby/TDM_FMCW_MIMO_RADAR/assets/99294980/5a21c806-7447-4aad-bce5-ad5b51f514f9)

Note that since each element in the transmit array radiates independently, there is no transmit beamforming, so the transmit pattern is broad and covers a large field of view (FOV). This allows the simultaneous illumination of all targets in the FOV. The receive array can then generate multiple beams to process all target echoes. Compared to conventional phased array radars that need successive scans to cover the entire FOV, this is another advantage of MIMO radars for applications that require fast reaction time.

Time division multiplexing (TDM) is one way to achieve orthogonality among transmit channels.

## Generating MIMO Radar Data Cube

The Radar's Field of View (FOV) has two cars at azimuth of 10 and -10 degrees. The MATLAB Phased Array System Toolbox. 

The Radar Data Cube corresponding to the physical array is first simulated and then is used to construct one generated from the virtual array.

![Screenshot 2023-05-25 014148-PhotoRoom](https://github.com/Mazen-Elaraby/TDM_FMCW_MIMO_RADAR/assets/99294980/c7185d53-aa2e-4f30-b10f-d9899c85c73f)

## The Signal Processing Chain
### Range-Doppler Processing

The Data Cube is sliced to pick one transmit-receive pair an d a 2D FFT is applied to produce the following range-doppler map:

![untitled2](https://github.com/Mazen-Elaraby/TDM_FMCW_MIMO_RADAR/assets/99294980/f969f635-39e4-4437-95c0-2559ae9478fc)

### Direction-of-Arrival (DOA) Estimation

Detection using a manual threshold deduced from the range-doppler map is performed. following that, range cuts of targets are extracted for further spatial processing.

The following is the estimate of the DOA of the two targets produced by the beamscanner algorithm:

![untitled3](https://github.com/Mazen-Elaraby/TDM_FMCW_MIMO_RADAR/assets/99294980/dced639a-16c9-47ea-8765-9aa77397c79c)

We can see how it successfully resolves both targets unlike the regular array.

## Dependencies 

This project requires the following dependencies:
- The Phased Array System Toolbox for MATLAB

## License

This project is licensed under the MIT License. See the LICENSE file for more information.

I hope this helps! Let me know if you have any questions.
