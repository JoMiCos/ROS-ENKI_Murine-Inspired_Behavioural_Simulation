#include "ctxneuron.h"
static float output2;

CtxNeuron::CtxNeuron(float _learningRateLTP, float _learningRateLTD, float _tLTD) : 
	learningRateLTP(_learningRateLTP),
	learningRateLTD(_learningRateLTD),
	tLTD(_tLTD),
	slowCaDetector(new SecondOrderLowpassFilter(_tLTD))
{
}
	


float CtxNeuron::doStep(float nonPlasticInput, float serot) {
	output = nonPlasticInput;
	for(int i=0;i<nInputs;i++) {
		output += weights[i] * *(inputs[i]);
	}
	if (output < 0) output = 0;
	if (output > 1) output = 1;
	if (serot < 0) serot = 0;
	output = ofc5HTreceptors(output,serot,serot*2);
		
	float dOutput = output - output2; //output2 not being remembered
	//printf("%f",output2);
	//printf("%s", "\n");
	if (dOutput < 0) dOutput = 0;

	float slowCa = slowCaDetector->filter(output);
	slowCa = fabs(slowCa);

	for(int i=0;i<nInputs;i++) {
		// weight change: LTP
		if (nonPlasticInput > 0.25)
			weightChange(weights[i], learningRateLTP * serot * *(inputs[i]) * dOutput);
		// weight change: LTD
		weightChange(weights[i], - tLTD * learningRateLTD * slowCa);
	}

	output2 = output;
	return output;
}
