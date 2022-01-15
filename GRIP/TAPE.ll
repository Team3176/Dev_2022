//
// Inputs
//
Inputs
{
	Mat source0;
}

//
// Variables
//
Outputs
{
	Mat hsvThresholdOutput;
}

//
// Steps
//

Step HSV_Threshold0
{
    Mat hsvThresholdInput = source0;
    List hsvThresholdHue = [74.46043165467624, 89.09090909090908];
    List hsvThresholdSaturation = [201.79856115107913, 255.0];
    List hsvThresholdValue = [57.32913669064748, 255.0];

    hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);
}




