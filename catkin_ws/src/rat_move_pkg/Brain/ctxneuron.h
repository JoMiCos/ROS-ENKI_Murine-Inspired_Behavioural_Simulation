#ifndef _CTXNEURON
#define _CTXNEURON

#include <math.h>
#include "filter.h"

// Based on Sjostrom (sjostrom_2002_plast_ca_ltp_ltd.pdf) and
// Ciranna (ciranna_2006_5ht_glu_receptors_review.pdf): Ca based conincidence detector
// Dayan / Iigaya 2018: 5HT controls plasticity, learning rate increased
// Nutt: 5HTR2 "Flexibilit of the mind" / plastcicity
// Kraus 2017: 5HTR2 predominantly in the PFC

class CtxNeuron {

public:
	CtxNeuron(float _learningRateLTP = 0.0, float _learningRateLTD = 0.0, float tLTD = 0.01);
		
	void addInput(float& anInput, float initWeight = 0) {
		if (nInputs == maxInputs) throw "No more inputs.";
		inputs[nInputs] = &anInput;
		weights[nInputs] = initWeight;
		nInputs++;
	}

	float doStep(float nonPlasticInput, float serot);

	float getWeight(int i) {
		return weights[i];
	}

	void setWeight(int i, float w) {
		weights[i] = w;
	}

private:
	float output;
	float output2;
	static const int maxInputs = 10;
	float* inputs[maxInputs];
	float weights[maxInputs];
	int nInputs = 0;
	float learningRateLTP;
	float learningRateLTD;
	float tLTD;
	SecondOrderLowpassFilter* slowCaDetector;
	
	void weightChange(float &w, float delta) {
		w += delta;
		if (w>1) w = 1;
		if (w<0) w = 0;
	}

public:
	static float ofc5HTreceptors(float x, float htR1, float htR2) {
		htR1 = 1 + htR1;
		htR2 = 2 + htR2;
		float r = (1-exp(-pow(x/htR1,htR1)))*htR2;
		if (r < 0.00001) return 0;
		return r;
	}

};

#endif

