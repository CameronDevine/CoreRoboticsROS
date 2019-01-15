//CREulerMode convention = CR_EULER_MODE_XYZ;

std::vector<CRFrameEuler*> frames = {
	new CRFrameEuler(0, 0, 128, 0, 0, 0, convention, CR_EULER_FREE_ANG_G),
	new CRFrameEuler(0, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_B),
	new CRFrameEuler(612.7, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_B),
	new CRFrameEuler(571.6, 163.9, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_B),
	new CRFrameEuler(0, 0, -115.7, 0, M_PI, 0, convention, CR_EULER_FREE_ANG_G),
	new CRFrameEuler(0, 92.2, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_B)};
