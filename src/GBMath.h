#pragma once

#include <cmath>
#include <algorithm>

// ----------------------------
// 3D Vector
// ----------------------------
struct GBVector3 {
    float x, y, z;

    // Constructors
    GBVector3() : x(0), y(0), z(0) {}
    GBVector3(float x, float y, float z) : x(x), y(y), z(z) {}

    // Operators
    GBVector3 operator+(const GBVector3& rhs) const { return { x + rhs.x, y + rhs.y, z + rhs.z }; }
    GBVector3 operator-(const GBVector3& rhs) const { return { x - rhs.x, y - rhs.y, z - rhs.z }; }
    GBVector3 operator*(float s) const { return { x * s, y * s, z * s }; }
    GBVector3 operator/(float s) const { return { x / s, y / s, z / s }; }
    GBVector3& operator+=(const GBVector3& rhs) { x += rhs.x; y += rhs.y; z += rhs.z; return *this; }
    GBVector3& operator-=(const GBVector3& rhs) { x -= rhs.x; y -= rhs.y; z -= rhs.z; return *this; }
    GBVector3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }
    GBVector3& operator/=(float s) { x /= s; y /= s; z /= s; return *this; }
    GBVector3 operator-() const { return { -x, -y, -z }; }
    GBVector3 operator*(const GBVector3& rhs) const { return { x * rhs.x, y * rhs.y, z * rhs.z }; }
    GBVector3& operator*=(const GBVector3& rhs) { x *= rhs.x; y *= rhs.y; z *= rhs.z; return *this; }

    // Utilities
    float length() const { return std::sqrt(x * x + y * y + z * z); }
    float lengthSquared() const { return x * x + y * y + z * z; }
    GBVector3 normalized() const { float l = length(); return l ? (*this / l) : GBVector3(0, 0, 0); }

    // Epsilon comparison
    bool epsilonEqual(const GBVector3& rhs, float epsilon = 1e-6f) const {
        return std::fabs(x - rhs.x) <= epsilon &&
            std::fabs(y - rhs.y) <= epsilon &&
            std::fabs(z - rhs.z) <= epsilon;
    }

    bool operator==(const GBVector3 other)
    {
        return x == other.x && y == other.y && z == other.z;
    }

    GBVector3& operator=(const GBVector3& rhs) {
        x = rhs.x;
        y = rhs.y;
        z = rhs.z;
        return *this;
    }

    static GBVector3 forward() { return { 1, 0, 0 }; } // X
    static GBVector3 right() { return { 0, 1, 0 }; } // Y
    static GBVector3 up() { return { 0, 0, 1 }; } // Z

    static GBVector3 back() { return { -1, 0, 0 }; }
    static GBVector3 left() { return { 0, -1, 0 }; }
    static GBVector3 down() { return { 0, 0, -1 }; }

    static GBVector3 zero() { return { 0,0,0 }; }
    static GBVector3 one() { return { 1,1,1 }; }

    float dot(GBVector3 other) const
    {
        return x * other.x + y * other.y + z * other.z;
    }

    GBVector3 cross(GBVector3 other) const
    {
        return { y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x };
    }

    GBVector3 clamped(float max)
    {
        float len = length();
        len = std::max(max, len);
        return *this / (float)len;
    }

    void truncate()
    {
        x = (int)x;
        y = (int)y; 
        z = (int)z;
    }

    GBVector3 xComponent()
    {
        return GBVector3(x, 0, 0);
    }

    GBVector3 yComponent()
    {
        return GBVector3(0, y, 0);
    }

    GBVector3 zComponent()
    {
        return GBVector3(0, 0, z);
    }

    GBVector3 xyComponent()
    {
        return GBVector3(x, y, 0);
    }

    GBVector3 xzComponent()
    {
        return GBVector3(x, 0, z);
    }

    GBVector3 yzComponent()
    {
        return GBVector3(0, y, z);
    }
};

constexpr float GB_PI = 3.14159265359f;

inline GBVector3 operator*(float s, const GBVector3& v)
{
    return { v.x * s, v.y * s, v.z * s };
}

// Free functions
inline float GBDot(const GBVector3& v0, const GBVector3& v1) {
    return v0.x * v1.x + v0.y * v1.y + v0.z * v1.z;
}

inline GBVector3 GBCross(const GBVector3& v0, const GBVector3& v1) {
    return { v0.y * v1.z - v0.z * v1.y, v0.z * v1.x - v0.x * v1.z, v0.x * v1.y - v0.y * v1.x };
}

inline GBVector3 GBNormalize(const GBVector3& v) {
    float len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    return (len > 1e-6f) ? GBVector3{ v.x / len, v.y / len, v.z / len } : GBVector3{ 0,0,0 };
}

inline GBVector3 GBAlign(const GBVector3& aligned, const GBVector3& v)
{
    GBVector3 result = aligned;
    if (GBDot(aligned, v) < 0.0f) // angle > 90 degrees
    {
        result = -aligned; // reverse
    }
    return result;
}

inline GBVector3 GBFixNormal(const GBVector3& normal, const GBVector3 pointOnPlane, const GBVector3 position)
{
	GBVector3 deltaPosition = pointOnPlane - position;
    if(GBDot(normal, deltaPosition) > 0.0f) // angle > 90 degrees
    {
        return -normal; // reverse
	}
    return normal;
}

inline bool GBIsAlign(const GBVector3& aligned, const GBVector3& v)
{
    if (GBDot(aligned, v) < 0.0f) // angle > 90 degrees
    {
        return false;
    }
    return true;
}

inline float GBClamp(float val, float minVal, float maxVal)
{
    if (val < minVal) return minVal;
    if (val > maxVal) return maxVal;
    return val;
}

inline float GBMin(float a, float b)
{
    return std::min(a, b);
}

inline float GBMax(float a, float b)
{
    return std::max(a, b);
}

static float GBEpsilon = 1e-5;
static float GBLargeEpsilon = 1e-4;

inline float GBAbs(float f)
{
    return std::fabs(f);
}

inline float GBSign(float f)
{
    return (f >= 0.0f) ? 1.0f : -1.0f;
}

inline GBVector3 GBSign(GBVector3 v)
{
    return GBVector3(GBSign(v.x), GBSign(v.y), GBSign(v.z));
}

inline bool GBEpsilonEquals(float a, float b, float epsilon = GBEpsilon)
{
    if (GBAbs(a - b) > epsilon)
        return true;
    return false;
}

inline GBVector3 GBMin(GBVector3 a, GBVector3 b)
{
    return GBVector3(GBMin(a.x, b.x), GBMin(a.y, b.y), GBMin(a.z, b.z));
}

inline GBVector3 GBMax(GBVector3 a, GBVector3 b)
{
    return GBVector3(GBMax(a.x, b.x), GBMax(a.y, b.y), GBMax(a.z, b.z));
}

inline GBVector3 GBClamp(GBVector3 a, GBVector3 min, GBVector3 max)
{
    return GBVector3(GBClamp(a.x, min.x, max.x), GBClamp(a.y, min.y, max.y), GBClamp(a.z, min.z, max.z));
}


inline GBVector3 GBAbs(GBVector3 v)
{
    return { GBAbs(v.x), GBAbs(v.y), GBAbs(v.z) };
}


inline GBVector3 GBLerp(const GBVector3& a, const GBVector3& b, float t)
{
    return a + (b - a) * t;
}


inline float GBComputeYawFromAxes(
    const GBVector3& right,   // Y axis
    const GBVector3& forward, // X axis
    const GBVector3& normal)
{
    GBVector3 ref = GBNormalize(right); // use right as reference
    float x = GBDot(forward, ref);      // projection of forward onto right
    float y = GBDot(forward, GBCross(normal, ref));
    return std::atan2f(y, x);
}


// ----------------------------
// 4D Vector
// ----------------------------
struct GBVector4 {
    float x, y, z, w;
    GBVector4() : x(0), y(0), z(0), w(0) {}
    GBVector4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}

    GBVector4 operator+(const GBVector4& rhs) const { return { x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w }; }
    GBVector4 operator-(const GBVector4& rhs) const { return { x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w }; }
    GBVector4 operator*(float s) const { return { x * s, y * s, z * s, w * s }; }
    GBVector4 operator/(float s) const { return { x / s, y / s, z / s, w / s }; }
};

// ----------------------------
// 4x4 Matrix
// ----------------------------
struct GBMatrix {
    float m[4][4];

    // Constructor: Identity
    GBMatrix() {
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                m[i][j] = (i == j) ? 1.0f : 0.0f;
    }

    // Matrix multiplication
    GBMatrix operator*(const GBMatrix& rhs) const {
        GBMatrix result;
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++) {
                result.m[i][j] = 0;
                for (int k = 0; k < 4; k++)
                    result.m[i][j] += m[i][k] * rhs.m[k][j];
            }
        return result;
    }

    // Transform a vector
    GBVector4 operator*(const GBVector4& v) const {
        return {
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z + m[0][3] * v.w,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z + m[1][3] * v.w,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z + m[2][3] * v.w,
            m[3][0] * v.x + m[3][1] * v.y + m[3][2] * v.z + m[3][3] * v.w
        };
    }

    // Static factories
    static GBMatrix translation(const GBVector3& t) {
        GBMatrix mat;
        mat.m[0][3] = t.x;
        mat.m[1][3] = t.y;
        mat.m[2][3] = t.z;
        return mat;
    }

    static GBMatrix scale(const GBVector3& s) {
        GBMatrix mat;
        mat.m[0][0] = s.x;
        mat.m[1][1] = s.y;
        mat.m[2][2] = s.z;
        return mat;
    }

    static GBMatrix rotationX(float angle) {
        GBMatrix mat;
        float c = cosf(angle), s = sinf(angle);
        mat.m[1][1] = c; mat.m[1][2] = -s;
        mat.m[2][1] = s; mat.m[2][2] = c;
        return mat;
    }

    static GBMatrix rotationY(float angle) {
        GBMatrix mat;
        float c = cosf(angle), s = sinf(angle);
        mat.m[0][0] = c; mat.m[0][2] = s;
        mat.m[2][0] = -s; mat.m[2][2] = c;
        return mat;
    }

    static GBMatrix rotationZ(float angle) {
        GBMatrix mat;
        float c = cosf(angle), s = sinf(angle);
        mat.m[0][0] = c; mat.m[0][1] = -s;
        mat.m[1][0] = s; mat.m[1][1] = c;
        return mat;
    }

    static GBMatrix rotationEuler(float pitch, float yaw, float roll)
    {
        // Unreal order: Z (Yaw) → Y (Pitch) → X (Roll)
        return rotationZ(yaw) * rotationY(pitch) * rotationX(roll);
    }

    static GBMatrix perspective(float fovRadians, float aspect, float zNear, float zFar) {
        float f = 1.0f / tanf(fovRadians * 0.5f);
        GBMatrix mat = {};
        mat.m[0][0] = f / aspect; mat.m[1][1] = f;
        mat.m[2][2] = (zFar + zNear) / (zNear - zFar);
        mat.m[2][3] = (2 * zFar * zNear) / (zNear - zFar);
        mat.m[3][2] = -1.0f; mat.m[3][3] = 0.0f;
        return mat;
    }

    static GBMatrix orthographic(float left, float right, float bottom, float top, float zNear, float zFar) {
        GBMatrix mat{};
        mat.m[0][0] = 2.0f / (right - left); mat.m[1][1] = 2.0f / (top - bottom);
        mat.m[2][2] = -2.0f / (zFar - zNear);
        mat.m[0][3] = -(right + left) / (right - left);
        mat.m[1][3] = -(top + bottom) / (top - bottom);
        mat.m[2][3] = -(zFar + zNear) / (zFar - zNear);
        return mat;
    }

    static GBMatrix lookAt(
        const GBVector3& eye,
        const GBVector3& target,
        const GBVector3& worldUp = { 0,0,1 }
    ) {
        GBVector3 forward = GBNormalize(target - eye);        // +Y
        GBVector3 right = GBNormalize(GBCross(worldUp, forward)); // +X
        GBVector3 up = GBCross(forward, right);          // +Z

        GBMatrix mat;
        mat.m[0][0] = right.x;   mat.m[0][1] = forward.x;   mat.m[0][2] = up.x;   mat.m[0][3] = -GBDot(right, eye);
        mat.m[1][0] = right.y;   mat.m[1][1] = forward.y;   mat.m[1][2] = up.y;   mat.m[1][3] = -GBDot(forward, eye);
        mat.m[2][0] = right.z;   mat.m[2][1] = forward.z;   mat.m[2][2] = up.z;   mat.m[2][3] = -GBDot(up, eye);
        mat.m[3][0] = 0;         mat.m[3][1] = 0;           mat.m[3][2] = 0;      mat.m[3][3] = 1;

        return mat;
    }

};

struct GBMatrix3
{
    float m[3][3];

    // Identity by default
    GBMatrix3()
    {
        m[0][0] = 1; m[0][1] = 0; m[0][2] = 0;
        m[1][0] = 0; m[1][1] = 1; m[1][2] = 0;
        m[2][0] = 0; m[2][1] = 0; m[2][2] = 1;
    }

    GBMatrix3(float a00, float a01, float a02,
        float a10, float a11, float a12,
        float a20, float a21, float a22)
    {
        m[0][0] = a00; m[0][1] = a01; m[0][2] = a02;
        m[1][0] = a10; m[1][1] = a11; m[1][2] = a12;
        m[2][0] = a20; m[2][1] = a21; m[2][2] = a22;
    }

    // Matrix × vector
    GBVector3 operator*(const GBVector3& v) const
    {
        return {
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
        };
    }

    // Matrix × matrix
    GBMatrix3 operator*(const GBMatrix3& rhs) const
    {
        GBMatrix3 r;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                r.m[i][j] = 0;
                for (int k = 0; k < 3; k++)
                    r.m[i][j] += m[i][k] * rhs.m[k][j];
            }
        return r;
    }

    // Transpose
    GBMatrix3 transposed() const
    {
        return GBMatrix3(
            m[0][0], m[1][0], m[2][0],
            m[0][1], m[1][1], m[2][1],
            m[0][2], m[1][2], m[2][2]
        );
    }

    // Extract diagonal as vector
    GBVector3 diagonal() const
    {
        return { m[0][0], m[1][1], m[2][2] };
    }

    // Replace the diagonal with a vector
    void setDiagonal(const GBVector3& d)
    {
        m[0][0] = d.x;
        m[1][1] = d.y;
        m[2][2] = d.z;
    }

    // Add to existing diagonal
    void addDiagonal(const GBVector3& d)
    {
        m[0][0] += d.x;
        m[1][1] += d.y;
        m[2][2] += d.z;
    }

    // Create a diagonal matrix
    static GBMatrix3 fromDiagonal(const GBVector3& d)
    {
        return GBMatrix3(
            d.x, 0, 0,
            0, d.y, 0,
            0, 0, d.z
        );
    }

    // Optimized inverse assuming diagonal matrix (inertia tensor)
    GBMatrix3 inverse() const
    {
        GBMatrix3 r;

        r.m[0][0] = 1.0f / m[0][0];
        r.m[1][1] = 1.0f / m[1][1];
        r.m[2][2] = 1.0f / m[2][2];

        r.m[0][1] = r.m[0][2] = 0;
        r.m[1][0] = r.m[1][2] = 0;
        r.m[2][0] = r.m[2][1] = 0;

        return r;
    }
};



// ----------------------------
// Quaternion
// ----------------------------
struct GBQuaternion {
    float w, x, y, z;

    // Constructors
    GBQuaternion() : w(1), x(0), y(0), z(0) {}
    GBQuaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}

    // Axis-angle
    static GBQuaternion fromAxisAngle(const GBVector3& axis, float angle) {
        float half = angle * 0.5f;
        float s = sinf(half);
        return { cosf(half), axis.x * s, axis.y * s, axis.z * s };
    }

    // Euler angles
    // Unreal: Yaw(Z) → Pitch(Y) → Roll(X)
    static GBQuaternion fromEuler(float pitch, float yaw, float roll)
    {
        float cr = cosf(roll * 0.5f);
        float sr = sinf(roll * 0.5f);
        float cp = cosf(pitch * 0.5f);
        float sp = sinf(pitch * 0.5f);
        float cy = cosf(yaw * 0.5f);
        float sy = sinf(yaw * 0.5f);

        GBQuaternion q;
        q.w = cy * cp * cr + sy * sp * sr;
        q.x = cy * cp * sr - sy * sp * cr;
        q.y = sy * cp * sr + cy * sp * cr;
        q.z = sy * cp * cr - cy * sp * sr;
        return q;
    }

    static GBQuaternion lookAt(
        const GBVector3& eye,
        const GBVector3& target,
        const GBVector3& worldUp = { 0,0,1 })
    {
        GBVector3 forward = GBNormalize(target - eye); // X
        GBVector3 right = GBNormalize(GBCross(worldUp, forward)); // Y
        GBVector3 up = GBCross(forward, right); // Z

        return GBQuaternion::fromAxes(forward, right, up);
    }



    // Quaternion multiplication
    GBQuaternion operator*(const GBQuaternion& q) const {
        return {
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        };
    }

    // Normalize
    void normalize() {
        float len = sqrtf(w * w + x * x + y * y + z * z);
        if (len > 1e-6f) { w /= len; x /= len; y /= len; z /= len; }
        else { w = 1; x = y = z = 0; }
    }

    GBQuaternion normalized() const {
        GBQuaternion q = *this;
        q.normalize();
        return q;
    }

    // Convert to 4x4 matrix
    GBMatrix toMatrix() const {
        GBMatrix mat;
        float xx = x * x, yy = y * y, zz = z * z;
        float xy = x * y, xz = x * z, yz = y * z;
        float wx = w * x, wy = w * y, wz = w * z;

        mat.m[0][0] = 1 - 2 * (yy + zz); mat.m[0][1] = 2 * (xy - wz); mat.m[0][2] = 2 * (xz + wy); mat.m[0][3] = 0;
        mat.m[1][0] = 2 * (xy + wz); mat.m[1][1] = 1 - 2 * (xx + zz); mat.m[1][2] = 2 * (yz - wx); mat.m[1][3] = 0;
        mat.m[2][0] = 2 * (xz - wy); mat.m[2][1] = 2 * (yz + wx); mat.m[2][2] = 1 - 2 * (xx + yy); mat.m[2][3] = 0;
        mat.m[3][0] = 0; mat.m[3][1] = 0; mat.m[3][2] = 0; mat.m[3][3] = 1;

        return mat;
    }

    // Rotate vector
    GBVector3 operator*(const GBVector3& v) const {
        GBVector3 qVec(x, y, z);
        GBVector3 t = GBCross(qVec, v) * 2.0f;
        return v + t * w + GBCross(qVec, t);
    }

    // Rotate local +Y (forward) to point at target (Unreal-style)
    static inline GBQuaternion lookAtForward(
        const GBVector3& from,
        const GBVector3& to,
        const GBVector3& worldUp = { 0,0,1 }
    ) {
        GBVector3 forward = GBNormalize(to - from);

        // Handle degenerate case: forward parallel to up
        if (fabs(GBDot(forward, worldUp)) > 0.9999f)
        {
            // Choose a stable fallback right axis
            GBVector3 right = GBCross({ 1,0,0 }, forward);
            if (right.length() < 1e-6f)
                right = GBCross({ 0,1,0 }, forward);

            right = GBNormalize(right);
            GBVector3 up = GBCross(forward, right); // forward × right = up

            // Use fromAxes with corrected order: forward=X, right=Y, up=Z
            return GBQuaternion::fromAxes(forward, right, up);
        }

        GBVector3 right = GBNormalize(GBCross(worldUp, forward));
        GBVector3 up = GBCross(forward, right);

        // Use fromAxes with corrected order: forward=X, right=Y, up=Z
        return GBQuaternion::fromAxes(forward, right, up);
    }



    GBVector3 rotate(const GBVector3& v) const {
        return (*this) * v;  // uses your existing operator*(vec3)
    }


    GBQuaternion& operator=(const GBQuaternion& rhs) {
        w = rhs.w;
        x = rhs.x;
        y = rhs.y;
        z = rhs.z;
        return *this;
    }

    GBQuaternion conjugate() const {
        return GBQuaternion(w, -x, -y, -z);
    }

    static GBQuaternion fromToRotation(const GBVector3& a, const GBVector3& b)
    {
        GBVector3 from = GBNormalize(a);
        GBVector3 to = GBNormalize(b);

        float cosTheta = GBDot(from, to);
        if (cosTheta >= 1.0f - 1e-6f)
        {
            // vectors are the same
            return GBQuaternion();
        }
        else if (cosTheta <= -1.0f + 1e-6f)
        {
            // vectors are opposite, pick an arbitrary perpendicular axis
            GBVector3 axis = GBCross(GBVector3(1, 0, 0), from);
            if (axis.length() < 1e-6f) axis = GBCross(GBVector3(0, 1, 0), from);
            axis = GBNormalize(axis);
            return GBQuaternion::fromAxisAngle(axis, 3.14159265f); // 180 degrees
        }
        else
        {
            GBVector3 axis = GBCross(from, to);
            float angle = acosf(cosTheta);
            axis = GBNormalize(axis);
            return GBQuaternion::fromAxisAngle(axis, angle);
        }
    }

    GBMatrix3 toMatrix3() const
    {
        GBMatrix3 r;

        float xx = x * x, yy = y * y, zz = z * z;
        float xy = x * y, xz = x * z, yz = y * z;
        float wx = w * x, wy = w * y, wz = w * z;

        r.m[0][0] = 1 - 2 * (yy + zz);
        r.m[0][1] = 2 * (xy - wz);
        r.m[0][2] = 2 * (xz + wy);

        r.m[1][0] = 2 * (xy + wz);
        r.m[1][1] = 1 - 2 * (xx + zz);
        r.m[1][2] = 2 * (yz - wx);

        r.m[2][0] = 2 * (xz - wy);
        r.m[2][1] = 2 * (yz + wx);
        r.m[2][2] = 1 - 2 * (xx + yy);

        return r;
    }


    static GBQuaternion fromAxes(GBVector3 forward, GBVector3 right, GBVector3 up)
    {
        // Orthonormalize
        GBVector3 f = GBNormalize(forward);
        GBVector3 r = GBNormalize(right - f * GBDot(f, right));
        GBVector3 u = GBCross(f, r);

        // Build 3x3 matrix (columns = forward=X, right=Y, up=Z)
        GBMatrix3 m;
        m.m[0][0] = f.x; m.m[0][1] = r.x; m.m[0][2] = u.x;
        m.m[1][0] = f.y; m.m[1][1] = r.y; m.m[1][2] = u.y;
        m.m[2][0] = f.z; m.m[2][1] = r.z; m.m[2][2] = u.z;

        // Convert matrix → quaternion
        float trace = m.m[0][0] + m.m[1][1] + m.m[2][2];
        GBQuaternion q;
        if (trace > 0.0f)
        {
            float s = sqrtf(trace + 1.0f) * 2.0f;
            q.w = 0.25f * s;
            q.x = (m.m[2][1] - m.m[1][2]) / s;
            q.y = (m.m[0][2] - m.m[2][0]) / s;
            q.z = (m.m[1][0] - m.m[0][1]) / s;
        }
        else if (m.m[0][0] > m.m[1][1] && m.m[0][0] > m.m[2][2])
        {
            float s = sqrtf(1.0f + m.m[0][0] - m.m[1][1] - m.m[2][2]) * 2.0f;
            q.w = (m.m[2][1] - m.m[1][2]) / s;
            q.x = 0.25f * s;
            q.y = (m.m[0][1] + m.m[1][0]) / s;
            q.z = (m.m[0][2] + m.m[2][0]) / s;
        }
        else if (m.m[1][1] > m.m[2][2])
        {
            float s = sqrtf(1.0f + m.m[1][1] - m.m[0][0] - m.m[2][2]) * 2.0f;
            q.w = (m.m[0][2] - m.m[2][0]) / s;
            q.x = (m.m[0][1] + m.m[1][0]) / s;
            q.y = 0.25f * s;
            q.z = (m.m[1][2] + m.m[2][1]) / s;
        }
        else
        {
            float s = sqrtf(1.0f + m.m[2][2] - m.m[0][0] - m.m[1][1]) * 2.0f;
            q.w = (m.m[1][0] - m.m[0][1]) / s;
            q.x = (m.m[0][2] + m.m[2][0]) / s;
            q.y = (m.m[1][2] + m.m[2][1]) / s;
            q.z = 0.25f * s;
        }

        q.normalize();
        return q;
    }

    void toAxisAngle(GBVector3& outAxis, float& outAngle) const
    {
        GBQuaternion q = *this;
        q.normalize();

        // Clamp to avoid acos NaNs
        float wClamped = GBClamp(q.w, -1.0f, 1.0f);

        outAngle = 2.0f * acosf(wClamped);

        float s = sqrtf(1.0f - wClamped * wClamped);

        if (s < 1e-6f)
        {
            // Angle ~ 0, axis arbitrary
            outAxis = GBVector3(1, 0, 0);
        }
        else
        {
            outAxis = GBVector3(q.x, q.y, q.z) / s;
        }

        // Wrap to shortest arc
        if (outAngle > 3.14159265f)
        {
            outAngle -= 2.0f * 3.14159265f;
            outAxis = -outAxis;
        }
    }

    static GBVector3 toAngularVelocity(const GBQuaternion& qOld,
        const GBQuaternion& qNew,
        float dt)
    {
        GBQuaternion dq = qNew * qOld.conjugate();
        dq.normalize();

        GBVector3 axis;
        float angle;
        dq.toAxisAngle(axis, angle);

        return axis * (angle / dt);
    }

    bool isIdentity(float eps = 1e-6f) const
    {
        return
            GBAbs(x) < eps &&
            GBAbs(y) < eps &&
            GBAbs(z) < eps &&
            GBAbs(w - 1.0f) < eps;
    }
};
