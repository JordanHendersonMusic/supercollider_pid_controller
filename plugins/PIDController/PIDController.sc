PIDController : UGen {
    const <pivar = 3.1415926535897932384626433832795028841971693993751058209749445923078164062;
	*ar {
         |target, freq, zeta, r|
		^this.multiNew('audio',
             target,
             zeta / (pivar * freq),
             1.0 / ((2 * pivar * freq) * (2 * pivar * freq)),
             r * zeta / (2 * pivar * freq)
             );
	}
	*kr {
         |target, freq, zeta, r|
		^this.multiNew('control',
             target,
             zeta / (pivar * freq),
             1.0 / ((2 * pivar * freq) * (2 * pivar * freq)),
             r * zeta / (2 * pivar * freq)
             );
	}

	checkInputs {
		^this.checkValidInputs;
	}
}
