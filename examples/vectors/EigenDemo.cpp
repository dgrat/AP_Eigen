/*
 * author: Daniel Frenzel <dgdanielf@gmail.com>
 */
#include <iostream>

#include <AP_Eigen/AP_Interface.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_Linux/AP_HAL_Linux.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>


const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;


AP_Eigen::Matrix3f rotate_a_to_b_i (const AP_Eigen::Vector3f &a, const AP_Eigen::Vector3f &b) {
    AP_Eigen::Vector3f a_norm = a.normalized();
    AP_Eigen::Vector3f b_norm = b.normalized();

    // Find axis and angle using cross product and ..
    AP_Eigen::Vector3f x = a_norm.cross(b_norm);
    
    // .. the dot product
    float theta = acosf( a_norm.dot(b_norm) );

    // A is a skew-symmetric matrix corresponding to x
    AP_Eigen::Matrix3f A( 0.f,    -x.z(),   x.y(),
                          x.z(),    0.f,    -x.x(),
                          -x.y(),   x.x(),    0.f);

    // Find rotation matrix using exponential map
    return AP_Eigen::Matrix3f::Identity() + sinf(theta) * A + ( (1 - cosf(theta) ) * (A*A) );
}

Matrix3f rotate_a_to_b_ii (const Vector3f &a, const Vector3f &b) {
    Vector3f a_norm = a.normalized();
    Vector3f b_norm = b.normalized();

    // Find axis and angle using cross product and ..
    Vector3f x = a_norm % b_norm;

    // .. the dot product
    float theta = acosf( a_norm * b_norm );

    // A is a skew-symmetric matrix corresponding to x
    Matrix3f A(0.f,    -x.z,   x.y,
               x.z,    0.f,    -x.x,
               -x.y,   x.x,    0.f);
    
    Matrix3f I; I.identity();
    
    // Find rotation matrix using exponential map
    return I + A * sinf(theta) + ( (A*A) * (1 - cosf(theta) ) );
}

// Memory
void setup(void) {
    /*
     * Test type size
     */
    int PV = sizeof(Eigen::Vector3f);
    int PM = sizeof(Eigen::Matrix3f);
  
    int EV = sizeof(AP_Eigen::Vector3f);
    int EM = sizeof(AP_Eigen::Matrix3f);
    
    int AV = sizeof(Vector3f);
    int AM = sizeof(Matrix3f);
    
    printf("Eigen:\t vector: %d\tmatrix: %d\n",  PV, PM);
    printf("Intf:\t vector: %d\tmatrix: %d\n",  EV, EM);
    printf("AP:\t vector: %d\tmatrix: %d\n",  AV, AM);
    
    /*
     * Test type operators
     */
    AP_Eigen::Vector3f v1(0,0,0);
    AP_Eigen::Vector3f v2(1,1,1);
    AP_Eigen::Vector3f v3(2,2,2);
    AP_Eigen::Vector3f v4(3.7,3.7,3.4);
    Eigen::Vector3f    v5(5.5,5.3,5.7);

    std::cout << "v1: " << v1 << std::endl;
    std::cout << "v2: " << v2 << std::endl;
    std::cout << "v3: " << v3 << std::endl;
    v1 = v2;
    v2 = v3 = v4;
    v4 = v5;
    std::cout << "new v1: " << v1 << std::endl;
    std::cout << "new v2: " << v2 << std::endl;
    std::cout << "new v3: " << v3 << std::endl;
    std::cout << "new v4: " << v4 << std::endl;

    AP_Eigen::Vector3i intv = static_cast<Vector3i>(v4);
    std::cout << "new intv: " << intv << std::endl;
}

// Benchmark
void loop(void)
{
    
    long timer1 = hal.scheduler->millis();
    AP_Eigen::Vector3f a(1,0,0);
    AP_Eigen::Vector3f b(0,1,99);
    
    AP_Eigen::Matrix3f m_i = rotate_a_to_b_i(a, b);
    AP_Eigen::Vector3f first = (m_i * a);
    AP_Eigen::Vector3f diff(0,0,0);
    
    for(int i = 0; i < 10000000; i++) {
        m_i = rotate_a_to_b_i(a, b);
        diff += (first - (m_i * a));
    }
    
    printf("Eigen:\t %f\t%f\t%f: %d\n",  diff.x(), diff.y(), diff.z(), hal.scheduler->millis() - timer1);

    long timer2 = hal.scheduler->millis();
    Vector3f c(1,0,0);
    Vector3f d(0,1,99);
    
    Matrix3f m_ii = rotate_a_to_b_ii(c, d);
    Vector3f first_ii = (m_ii * c);
    Vector3f diff_ii(0,0,0);
    
    for(int i = 0; i < 10000000; i++) {
        m_ii = rotate_a_to_b_ii(c, d);
        diff_ii += (first_ii - (m_ii * c));
    }
    
    printf("AP:\t %f\t%f\t%f: %d\n",  diff_ii.x, diff_ii.y, diff_ii.z, hal.scheduler->millis() - timer2);
}

AP_HAL_MAIN();
