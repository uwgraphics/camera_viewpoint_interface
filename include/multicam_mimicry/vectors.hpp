#ifndef __VECTORS_HPP__
#define __VECTORS_HPP__

#include <string>

namespace goal_publisher
{

struct Vec3;
struct Vec4;
struct EulerAngles;

struct Vec3
{
    union {
        struct {
            float x, y, z;
        };
        float vals[3];
    };

    Vec3(float x_val, float y_val, float z_val) : x(x_val), y(y_val), z(z_val) {}
    Vec3() : x(0.0), y(0.0), z(0.0) {}
    Vec3(const Vec3& v) : x(v.x), y(v.y), z(v.z) {}

    Vec3& operator +=(const Vec3& v);
    Vec3& operator -=(const Vec3& v);
    Vec3& operator *=(float s);

    std::string to_str(uint ind=0);
};

const Vec3 operator +(const Vec3& a, const Vec3& b);
const Vec3 operator -(const Vec3& a, const Vec3& b);
const Vec3 operator *(float s, const Vec3& v);
const Vec3 operator *(const Vec3& v, float s);


struct Vec4
{
    union {
        struct {
            float x, y, z, w;
        };
        float vals[4];
    };
        
    Vec4(float x_val, float y_val, float z_val, float w_val) : x(x_val), y(y_val), z(z_val), 
        w(w_val) {}
    Vec4() : x(0.0), y(0.0), z(0.0), w(1.0) {}
    Vec4(const Vec4& v) : x(v.x), y(v.y), z(v.z), w(v.w) {}

    Vec4& operator +=(const Vec4& v);
    Vec4& operator -=(const Vec4& v);
    Vec4& operator *=(float s);
    const Vec4 conjugate() const;
    const float norm() const;
    const float magnitude() const;
    const Vec4 inverse() const;
    const Vec4 operator *(const Vec4& a) const;
    const EulerAngles toEulerAngles() const;
    std::string to_str(uint ind=0, bool show_euler=false) const;
};

const Vec4 operator +(const Vec4& a, const Vec4& b);
const Vec4 operator -(const Vec4& a, const Vec4& b);
const Vec4 operator *(float s, const Vec4& v);
const Vec4 operator *(const Vec4& v, float s);


struct EulerAngles
{
    union {
        struct {
            double roll, pitch, yaw;
        };
        double vals[3];
    };

    EulerAngles(double r, double p, double y) : roll(r), pitch(p), yaw(y) {}
    EulerAngles() : roll(0.0), pitch(0.0), yaw(0.0) {}
    EulerAngles(const Vec3& v) : roll(v.x), pitch(v.y), yaw(v.z) {}

    const Vec4 toQuaternion() const;
};


const float dot(const Vec3& a, const Vec3& b);
const Vec3 cross(const Vec3& a, const Vec3& b);
const Vec3 rotate_vector(const Vec3& v, const Vec4& q);
const Vec4 DispQ(const Vec4& q1, const Vec4& q2);
const EulerAngles radToDeg(const EulerAngles& euler_vec);

} // goal_publisher


#endif // __VECTORS_HPP__