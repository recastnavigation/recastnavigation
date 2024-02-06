#include "Deterministic.h"

const int SIGN = 0x80000000;

int as_int(float f) {
    return *reinterpret_cast<int*>(&f);
}

float as_float(int i) {
    return *reinterpret_cast<float*>(&i);
}

DmSinCos dmSinCos(float n) {
    // Implementation based on Vec4.inl from the JoltPhysics
    // https://github.com/jrouwe/JoltPhysics/blob/master/Jolt/Math/Vec4.inl

    const float N1 = 1.5703125;
    const float N2 = 0.0004837512969970703125;
    const float N3 = 7.549789948768648e-8;

    const float C1 = 2.443315711809948e-5;
    const float C2 = 1.388731625493765e-3;
    const float C3 = 4.166664568298827e-2;

    const float S1 = -1.9515295891e-4;
    const float S2 = 8.3321608736e-3;
    const float S3 = 1.6666654611e-1;

    // Make argument positive and remember sign for sin only since cos is symmetric around x (highest bit of a float is the sign bit)
    int sin_sign = int(n < 0.0) << 31;
    float x = fabsf(n);

    // x / (PI / 2) rounded to nearest int gives us the quadrant closest to x
    int quadrant = (int)(FRAC_2_PI * x + 0.5f);

    // Make x relative to the closest quadrant.
    // This does x = x - quadrant * PI / 2 using a two step Cody-Waite argument reduction.
    // This improves the accuracy of the result by avoiding loss of significant bits in the subtraction.
    // We start with x = x - quadrant * PI / 2, PI / 2 in hexadecimal notation is 0x3fc90fdb, we remove the lowest 16 bits to
    // get 0x3fc90000 (= 1.5703125) this means we can now multiply with a number of up to 2^16 without losing any bits.
    // This leaves us with: x = (x - quadrant * 1.5703125) - quadrant * (PI / 2 - 1.5703125).
    // PI / 2 - 1.5703125 in hexadecimal is 0x39fdaa22, stripping the lowest 12 bits we get 0x39fda000 (= 0.0004837512969970703125)
    // This leaves uw with: x = ((x - quadrant * 1.5703125) - quadrant * 0.0004837512969970703125) - quadrant * (PI / 2 - 1.5703125 - 0.0004837512969970703125)
    // See: https://stackoverflow.com/questions/42455143/sine-cosine-modular-extended-precision-arithmetic
    // After this we have x in the range [-PI / 4, PI / 4].
	float float_quadrant = float(quadrant);
    x = ((x - float_quadrant * N1) - float_quadrant * N2) - float_quadrant * N3;

    // Calculate x2 = x^2
    float x2 = x * x;

    // Taylor expansion:
    // Cos(x) = 1 - x^2/2! + x^4/4! - x^6/6! + x^8/8! + ... = (((x2/8!- 1/6!) * x2 + 1/4!) * x2 - 1/2!) * x2 + 1
    float taylor_cos = ((C1 * x2 - C2) * x2 + C3) * x2 * x2 - 0.5f * x2 + 1.0f;
    // Sin(x) = x - x^3/3! + x^5/5! - x^7/7! + ... = ((-x2/7! + 1/5!) * x2 - 1/3!) * x2 * x + x
    float taylor_sin = ((S1 * x2 + S2) * x2 - S3) * x2 * x + x;

    // The lowest 2 bits of quadrant indicate the quadrant that we are in.
    // Let x be the original input value and x' our value that has been mapped to the range [-PI / 4, PI / 4].
    // since cos(x) = sin(x - PI / 2) and since we want to use the Taylor expansion as close as possible to 0,
    // we can alternate between using the Taylor expansion for sin and cos according to the following table:
    //
    // quadrant	 sin(x)		 cos(x)
    // XXX00b	 sin(x')	 cos(x')
    // XXX01b	 cos(x')	-sin(x')
    // XXX10b	-sin(x')	-cos(x')
    // XXX11b	-cos(x')	 sin(x')
    //
    // So: sin_sign = bit2, cos_sign = bit1 ^ bit2, bit1 determines if we use sin or cos Taylor expansion
    int bit1 = quadrant << 31;
    int bit2 = (quadrant << 30) & SIGN;

    // Select which one of the results is sin and which one is cos
    float s = bit1 ? taylor_cos : taylor_sin;
    float c = bit1 ? taylor_sin : taylor_cos;

    // Update the signs
    sin_sign = sin_sign ^ bit2;
    int cos_sign = bit1 ^ bit2;

    // Correct the signs
    DmSinCos res;
    res.sin = as_float(as_int(s) ^ sin_sign);
    res.cos = as_float(as_int(c) ^ cos_sign);
    return res;
}

float dmASin(float n) {
    // Implementation based on Vec4.inl from the JoltPhysics
    // https://github.com/jrouwe/JoltPhysics/blob/master/Jolt/Math/Vec4.inl

    const float N1 = 4.2163199048e-2;
    const float N2 = 2.4181311049e-2;
    const float N3 = 4.5470025998e-2;
    const float N4 = 7.4953002686e-2;
    const float N5 = 1.6666752422e-1;

    // Make argument positive
    int asin_sign = int(n < 0.0f) << 31;
    float a = fabsf(n);

    // ASin is not defined outside the range [-1, 1] but it often happens that a value is slightly above 1 so we just clamp here
    a = fminf(a, 1.0f);

    // When |x| <= 0.5 we use the asin approximation as is
    float z1 = a * a;
    float x1 = a;

    // When |x| > 0.5 we use the identity asin(x) = PI / 2 - 2 * asin(sqrt((1 - x) / 2))
    float z2 = 0.5f * (1.0f - a);
    float x2 = sqrtf(z2);

    // Select which of the two situations we have
    bool greater = a > 0.5f;
    float z = greater ? z2 : z1;
    float x = greater ? x2 : x1;

    // Polynomial approximation of asin
    z = ((((N1 * z + N2) * z + N3) * z + N4) * z + N5) * z * x + x;

    // If |x| > 0.5 we need to apply the remainder of the identity above
    z = greater ? FRAC_PI_2 - (z + z) : z;

    // Put the sign back
    return as_float(as_int(z) ^ asin_sign);
}
