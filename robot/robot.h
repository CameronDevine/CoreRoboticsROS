CREulerMode convention = CR_EULER_MODE_XYZ;

std::vector<CR::CRFrameEuler*> frames = {
	new CRFrameEuler(0, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_G),
	new CRFrameEuler(0, 0, 89.2, 0, 0, 0, convention, CR_EULER_FREE_ANG_B),
	new CRFrameEuler(425, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_B),
	new CRFrameEuler(392, 109.3, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_B),
	new CRFrameEuler(94.75, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_A),
	new CRFrameEuler(0, 82.5, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_B)};

