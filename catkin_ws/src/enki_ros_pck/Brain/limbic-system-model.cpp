#include"limbic-system-model.h"

#include <stdio.h>
#include <iostream>
#include <filter.h>

float LHB_BIAS = 0;
float DRN_SUPPRESSION = 0;
float OFC_5HTR1_OFFSET = 0;
float OFC_5HTR2_OFFSET = 0;
float DRN_OFFSET = 0;
static float rm = RAND_MAX;
static int exploreState = 0;
static bool limbic_count{0};
//static int step;


// constructor
Limbic_system::Limbic_system()
{	
	switch (limbic_count)
	{
	case 0:
		flog = fopen("limbic-log.dat","w");
		limbic_count = 1;
		break;
	
	default:
		flog = fopen("limbic-log.dat","a");
		break;
	}
	
	

	// filter which creates a default behaviour when touching the landmarks
	on_contact_direction_Green_filter = new SecondOrderLowpassFilter(0.1);
	on_contact_direction_Blue_filter = new SecondOrderLowpassFilter(0.1);

	// filter which smears out the visual input in time as a 1st attempt
	// to create persistent activity in the cortex
	visual_direction_Green_mPFC_filter = new SecondOrderLowpassFilter(0.01);
	visual_direction_Blue_mPFC_filter = new SecondOrderLowpassFilter(0.01);

	// self inhibition VTA
	VTA_forwardinhibition = new SecondOrderLowpassFilter(0.01);

	placefield_Green_filter = new SecondOrderLowpassFilter(0.1);
	placefield_Blue_filter = new SecondOrderLowpassFilter(0.1);

	reward_filter = new SecondOrderLowpassFilter(0.5);

	DRNto5HTrelease = new SecondOrderLowpassFilter(0.001);

	OFCNeuron = new CtxNeuron(learning_rate_OFC,learning_rate_OFC * 0.1);
	// OFCNeuron->addInput(placefieldGreen);
	// OFCNeuron->addInput(placefieldBlue);
	OFCNeuron->addInput(visual_direction_Green_trace);
	OFCNeuron->addInput(visual_direction_Blue_trace);

	const float w = 1;
	mPFCneuronGreen = new CtxNeuron(learning_rate_mPFC,learning_rate_mPFC*0.01); //making neurons using neuron template, each has different effect weight
	mPFCneuronGreen->addInput(visual_direction_Green_trace,w*0.25);
	
	mPFCneuronBlue = new CtxNeuron(learning_rate_mPFC,learning_rate_mPFC*0.01);
	mPFCneuronBlue->addInput(visual_direction_Blue_trace,w*0.25);
    mPFCneuronBlue->addInput(visual_reward_Blue,w);	 
		
	// input step number
	//step = 0;
};


// destructor
Limbic_system::~Limbic_system()
{
	fclose(flog);
}


// changes a synaptic weight "w" by the amount "delta"
// and makes sure it never goes below 0 and above 1
void Limbic_system::weightChange(float &w, float delta) {
	w += delta;
	if (w>1) w = 1;
	if (w<0) w = 0;
}

// that's what our limbic system does!
// remember there are two landmarks/place fields: light green and dark green
// only one of them contains the reward (controlled by the main program)
//
// It gets as inputs:
// - the reward
// - two place field signals (placefield*) which go from 0 to 1 when in the place field (where the reward shows up)
// - two signals when the agent touches the landmark: (on_contact_direction_*)
// - visual inputs when the agent sees a landmark which goes from 0 to 1 the closer the agent gets (_visual_direction_*)
// - visual inputs when the agent sees the reward (_visual_reward_*)
//
// It needs to set the outputs:
// - CoreGreenOut and CoreBlueOut which when set to non-zero generates a navigation behaviour towards
//   the landmarks
// - mPFC2CoreExploreLeft and - mPFC2CoreExploreRight to generate exploration behaviour
//   that is inhibited with other activities.
void Limbic_system::doStep(uint32_t _step, float _reward,
		float _placefieldGreen,
		float _placefieldBlue,
		float _on_contact_direction_Green,
		float _on_contact_direction_Blue,
		float _visual_direction_Green,
		float _visual_direction_Blue,
		float _visual_reward_Green,
		float _visual_reward_Blue ) {

	step = _step;
	reward = reward_filter->filter(_reward);
	if (reward > 0)
	{
		printf("%f", _reward);
		printf("%s", "\n");
		printf("%f", reward);
		printf("%s", "\n");
	}
	placefieldGreen = placefield_Green_filter->filter(_placefieldGreen);
	placefieldBlue = placefield_Blue_filter->filter(_placefieldBlue);
	on_contact_direction_Green = _on_contact_direction_Green;
	on_contact_direction_Blue = _on_contact_direction_Blue;
	visual_direction_Green = _visual_direction_Green;
	visual_direction_Blue = _visual_direction_Blue;
	visual_reward_Green = _visual_reward_Green;
	visual_reward_Blue = _visual_reward_Blue;
	//fprintf(stderr,"%f,%f\n",_visual_reward_Green,_visual_reward_Blue);


	visual_direction_Green_trace = visual_direction_Green_mPFC_filter->filter(visual_direction_Green);
	visual_direction_Blue_trace = visual_direction_Blue_mPFC_filter->filter(visual_direction_Blue);

	////////////////////////////////////////////////////////////////////
	// OFC
	// lace field -> Orbitofrontal Cortex weight.
	// The OFC remembers the reward value of a place.
	// So the higher the weight the higher the OFC activity
	// when the animal is inside that place field where there has been
	// reward experienced in the past.
	// It codes reward value and the primary reward also has a
	// value.
	OFC = OFCNeuron->doStep(reward, serotoninConcentration+0.1);
	if (OFC > 0.25) {
		OFC = 0.25;
	}
	
	// LH
	LH = OFC;

	// DRN
	// the dorsal raphe activity is driven by the OFC in a positive way
	DRN = (LH + OFC * 4) / (1+RMTg * shunting_inhibition_factor + DRN_SUPPRESSION) + DRN_OFFSET;

	serotoninConcentration = DRNto5HTrelease->filter(DRN);

	// the VTA gets its activity from the LH and is ihibited by the RMTg
	VTA = (LH + VTA_baseline_activity) / (1+(RMTg + VTA_forwardinhibition->filter(OFC*0.1)) * shunting_inhibition_factor);

	// this is also generated in the mPFC and then fed down to the NAcc core with the command
	// to explore
	//edit these so that they act on rat reversal, options:
	// write publisher straight into here.. dont think it works without a main(), maybe it does if its called in a file with main
	//have this write
	switch (exploreState) {
	case EXPLORE_LEFT:
		mPFC2CoreExploreLeft = 0.1;
		mPFC2CoreExploreRight = 0;
		if ((random()/(rm))<0.03) {
			//printf("left\n");
			exploreState = floor(4*(random()/rm));
		}
		//printf("left\n");
		break;
	case EXPLORE_RIGHT:
		mPFC2CoreExploreLeft = 0;
		mPFC2CoreExploreRight = 0.1;
		if ((random()/(rm))<0.03) {
			exploreState = floor(4*(random()/rm));
		}
		//printf("right\n");
		break;
	case EXPLORE_STOP:
		mPFC2CoreExploreLeft = 0;
		mPFC2CoreExploreRight = 0;
		if ((random()/(rm))<0.1) {
			exploreState = floor(4*(random()/rm));
		}
		//printf("stop\n");
		break;
	case EXPLORE_STRAIGHT:
	default:
		mPFC2CoreExploreLeft = 0.1;
		mPFC2CoreExploreRight = 0.1;
		if ((random()/(rm))<0.05) {
			exploreState = floor(4*(random()/rm));
			//printf("%d", exploreState);
			//printf("\n");
		}
		//printf("straight\n");
		break;
	}

	mPFC_Green = mPFCneuronGreen->doStep(visual_reward_Green + OFC, serotoninConcentration);
	mPFC_Blue = mPFCneuronBlue->doStep(visual_reward_Blue + OFC, serotoninConcentration);

	// printf("%f %f\n",visual_reward_Green,mPFC_Green);

        ////////////////////////////////////////////////////
	// lateral shell activity
	// the place field feeds into the Nacc shell for the time being.
	lShell = placefieldGreen * lShell_weight_pflg + placefieldBlue * lShell_weight_pfdg;

	// Let's do heterosynaptic plasticity in the shell
	shell_DA = VTA;
	// shell plasticity can experience a "dip" where then the weights
	// decrease. That's when it's below its baseline.
	shell_plasticity = shell_DA - VTA_zero_val;
	weightChange(lShell_weight_pflg, learning_rate_lshell * shell_plasticity * placefieldGreen);
	weightChange(lShell_weight_pfdg, learning_rate_lshell * shell_plasticity * placefieldBlue);

	// the shell inhibits the dlVP
	dlVP = 1/(1+lShell * shunting_inhibition_factor);

	// another inhibition: the dlVP inhibits the EP
	EP = 1/(1+dlVP * shunting_inhibition_factor);

	// the EP excites the LHb
	LHb = EP + LHB_BIAS;

	// and the LHb excites the RMTg
	RMTg = LHb;

	////////////////////////////////////////////////////////////////////////////////////
	// core
	// we have two core units
	// if the Green is high then the rat approaches the Green marker
	CoreGreenOut= (mPFC_Green * core_weight_lg2lg);
	// of the Blue is high then the rat approaches the Blue marker
	CoreBlueOut= (mPFC_Blue * core_weight_dg2dg);

	// fprintf(stderr,"%f %f\n",CoreGreenOut,CoreBlueOut);

	float core_threshold = 0.01;
	if (CoreGreenOut < core_threshold) CoreGreenOut = 0;
	if (CoreBlueOut < core_threshold) CoreBlueOut = 0;

	if (CoreGreenOut > 1) CoreGreenOut = 1;
	if (CoreBlueOut > 1) CoreBlueOut = 1;


	// plasticity
	core_DA = VTA;
	core_plasticity = core_DA - VTA_zero_val;
	weightChange(core_weight_lg2lg, learning_rate_core * core_plasticity * mPFC_Green);
	weightChange(core_weight_dg2dg, learning_rate_core * core_plasticity * mPFC_Blue);

	// we assume that the Core performs lateral inhibtion to shut down exploration
		//if ((CoreGreenOut > 0.05)||(CoreBlueOut > 0.05)) {
		//mPFC2CoreExploreLeft = 0; // (float)random()/(float)RAND_MAX;
		//mPFC2CoreExploreRight = 0; //(float)random()/(float)RAND_MAX;
		//printf("dir! mPFC_Green = %f\n",visual_direction_Green);
	//}
	//printf("%d", step);
	//printf("%f", mPFCneuronGreen->getWeight(0));
	//printf("%s", "\n");
	logging();

	//step++;  

}	




void Limbic_system::logging() {
	fprintf(flog,"%d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f "
		"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
		step, //0
		reward, //1
		placefieldGreen, //2
		placefieldBlue, //3
		on_contact_direction_Green, //4
		on_contact_direction_Blue, //5
		visual_direction_Green_trace, //6
		visual_direction_Blue_trace, //7
		core_weight_lg2lg, //8
		mPFC_Green, //9
		mPFC_Blue, //10
		core_weight_dg2dg, //11
		CoreGreenOut, //12
		CoreBlueOut, //13
		VTA,//14
		core_plasticity,//15
		shell_plasticity,//16
		lShell_weight_pflg,//17
		lShell_weight_pfdg,//18
		dlVP,//19
		EP,//20
		LHb,//21
		RMTg,//22
		pfLg2OFC,//23
		pfDg2OFC, //24
		DRN, //25
		visual_reward_Green, // 26
		visual_reward_Blue, //27
		OFC, // 28
		serotoninConcentration, // 29
		OFCNeuron->getWeight(0), // 30
		OFCNeuron->getWeight(1), // 31
		mPFCneuronGreen->getWeight(0), // 32
		mPFCneuronBlue->getWeight(0) // 33
		);
	fflush(flog);
}

