# Fast and Automated Optical Polarization Compensation of Fiber Unitaries
We present a fast and automated method for polarization compensation using liquid crystals. This approach combines rotating quarter-waveplate polarimetry with liquid crystal cell voltage adjustments, offering high-fidelity compensation suitable for diverse applications. Our method directly solves for compensation parameters, avoiding reliance on stochastic approaches or cryptographic metrics. Check our paper for further details at: ...

# Note about LCVR characterization files
The `LCVR_characterization.py` should be used to generate the characterization data for each LCVR. The output files should then be processed using `LCVR_retardance_calculation.py` to obtain the voltage-retardance relation. The .csv files generated through `LCVR_retardance_calculation.py` are to be used in the compensation algorithm. They should be renamed/relocated appropriately before running `Polarisation_Compensation.py`.
