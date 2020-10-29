
#include <string>
#include <cmath>

#include <iostream> // TEST

#include "openvr_ros/vectors.hpp"

namespace goal_publisher
{

// TEST
void printText(std::string text="", int newlines=1, bool flush=false)
{
    // TODO: Consider adding param for width of text line
    std::cout << text;

    for (int i = 0; i < newlines; i++) {
        if (i > 1) {
            std::cout << "\n";
        }
        else {
            std::cout << std::endl;
        }
    }

    if (flush) {
        std::cout.flush();
    }

    return;
}


// Vec3

Vec3& Vec3::operator +=(const Vec3& v)
{
    x += v.x;
    y += v.y;
    z += v.z;

    return *this;
}

Vec3& Vec3::operator -=(const Vec3& v)
{
    x -= v.x;
    y -= v.y;
    z -= v.z;

    return *this;
}

Vec3& Vec3::operator *=(float s)
{
    x *= s;
    y *= s;
    z *= s;

    return *this;
}

const Vec3 operator +(const Vec3& a, const Vec3& b)
{
    return Vec3(a) += b;
}

const Vec3 operator -(const Vec3& a, const Vec3& b)
{
    return Vec3(a) -= b;
}

const Vec3 operator *(float s, const Vec3& v)
{
    return Vec3(v) *= s;
}

 const Vec3 operator *(const Vec3& v, float s)
{
    return Vec3(v) *= s;
}

std::string Vec3::to_str(uint ind)
{
    std::string indent = "";
    for (int i = 0; i < ind; i++) {
        indent += "\t";
    }

    std::string content;
    content  = indent + "x: " + std::to_string(x) + "\n";
    content += indent + "y: " + std::to_string(y) + "\n";
    content += indent + "z: " + std::to_string(z) + "\n";

    return content;
}


// Vec4

Vec4& Vec4::operator +=(const Vec4& v)
{
    x += v.x;
    y += v.y;
    z += v.z;
    w += v.w;

    return *this;
}

Vec4& Vec4::operator -=(const Vec4& v)
{
    x -= v.x;
    y -= v.y;
    z -= v.z;
    w -= v.w;

    return *this;
}

Vec4& Vec4::operator *=(float s)
{
    x *= s;
    y *= s;
    z *= s;
    w *= s;

    return *this;
}

const Vec4 operator +(const Vec4& a, const Vec4& b)
{
    return Vec4(a) += b;
}

const Vec4 operator -(const Vec4& a, const Vec4& b)
{
    return Vec4(a) -= b;
}


const Vec4 Vec4::conjugate() const
{
    Vec4 result;

    result.x = -x;
    result.y = -y;
    result.z = -z;
    result.w =  w;

    return result;
}

const float Vec4::norm() const
{
    return x*x + y*y + z*z + w*w;
}

const float Vec4::magnitude() const
{
    return sqrt(norm());
}

const Vec4 Vec4::inverse() const
{
    return conjugate() * (1.0f / norm());
}

const Vec4 Vec4::operator *(const Vec4& a) const 
{
    Vec4 result;

    result.x = w*a.x + x*a.w + y*a.z - z*a.y;                          
    result.y = w*a.y + y*a.w + z*a.x - x*a.z;
    result.z = w*a.z + z*a.w + x*a.y - y*a.x;
    result.w = w*a.w - x*a.x - y*a.y - z*a.z;

    return result;
}

const EulerAngles Vec4::toEulerAngles() const
{
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w*x + y*z);
    double cosr_cosp = 1 - 2 * (x*x + y*y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w*y - z*x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w*z + x*y);
    double cosy_cosp = 1 - 2 * (y*y + z*z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

std::string Vec4::to_str(uint ind, bool show_euler) const
{
    std::string indent = "";
    for (int i = 0; i < ind; i++) {
        indent += "\t";
    }

    EulerAngles euler;
    EulerAngles deg_euler;
    if (show_euler) {
        euler = toEulerAngles();
        deg_euler = radToDeg(euler);
    }

    std::string content;
    content  = indent + "x: " + std::to_string(x);
    if (show_euler) {
        content += "\tRoll: " + std::to_string(euler.roll) + " (" + std::to_string(deg_euler.roll) + ")";
    }
    content += "\n";
    content += indent + "y: " + std::to_string(y);
    if (show_euler) {
        content += "\tPitch: " + std::to_string(euler.pitch) + " (" + std::to_string(deg_euler.pitch) + ")";
    }
    content += "\n";
    content += indent + "z: " + std::to_string(z);
    if (show_euler) {
        content += "\tYaw: " + std::to_string(euler.yaw) + " (" + std::to_string(deg_euler.yaw) + ")";
    }
    content += "\n";
    content += indent + "w: " + std::to_string(w) + "\n";

    return content;
}

const Vec4 operator *(float s, const Vec4& v)
{
    return Vec4(v) *= s;
}

const Vec4 operator *(const Vec4& v, float s)
{
    return Vec4(v) *= s;
}


// EulerAngles

const Vec4 EulerAngles::toQuaternion() const
{
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Vec4 q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}


// General use function

const float dot(const Vec3& a, const Vec3& b)
{
    return (a.x*b.x) + (a.y*b.y) + (a.z*b.z);
}

const Vec3 cross(const Vec3& a, const Vec3& b)
{
    Vec3 result;

    result.x = a.y*b.z - a.z*b.y;
    result.y = a.x*b.z - a.z*b.x;
    result.z = a.x*b.y - a.y*b.x;

    return result;
}

const Vec3 rotate_vector(const Vec3& v, const Vec4& q)
{
    Vec3 u(q.x, q.y, q.z);
    float s(q.w);

    Vec3 result = (2.0f * dot(u, v) * u)
                + ((s*s - dot(u, u)) * v)
                + (2.0f * s * cross(u, v));

    return result;
}

const Vec4 DispQ(const Vec4& q1, const Vec4& q2)
{
    q1.inverse() * q2;
}

const EulerAngles radToDeg(const EulerAngles& euler_vec)
{
    EulerAngles deg_vec;

    for (int i = 0; i < 3; i++) {
        deg_vec.vals[i] = euler_vec.vals[i] * (180/M_PI);
    }

    return deg_vec;
}


} // goal_publisher
