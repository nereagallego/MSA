/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>
#include <cmath>

NORI_NAMESPACE_BEGIN

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

Point2f Warp::squareToTent(const Point2f &sample) {
    throw NoriException("Warp::squareToTent() is not yet implemented!");
}

float Warp::squareToTentPdf(const Point2f &p) {
    throw NoriException("Warp::squareToTentPdf() is not yet implemented!");
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    float r = sqrt(sample.x());
    float theta = 2 * M_PI * sample.y();

    float x = r * cos(theta);
    float y = r * sin(theta);

    return Point2f(x, y);
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    float r = p.norm();
    return r <= 1 ? 1 / M_PI : 0;
}

Point2f Warp::squareToUniformTriangle(const Point2f& sample) {
    if (sample.x() + sample.y() > 1) {
        return Point2f(1 - sample.x(), 1 - sample.y());
    }
    return Point2f(sample.x(), sample.y());
}

float Warp::squareToUniformTrianglePdf(const Point2f& p) {
    return (p.x() >= 0 && p.y() >= 0 && p.x() <= 1.0 && p.y() <= 1.0 && p.y() + p.x() <= 1.0) ? 2 : 0;
}


Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    float theta = 2 * M_PI * sample.x();
    float phi = acos(2 * sample.y() - 1);

    return Vector3f(sin(phi)*cos(theta), sin(phi)*sin(theta), cos(phi));
   
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    return 1 / (4*M_PI);
}

float max(float a, float b){
    return a ? a > b : b;
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    float z = sample.x();
    float r = sqrt(max(0,1-z*z));
    float phi = 2 * M_PI * sample.y();
    return Vector3f(r*cos(phi), r * sin(phi),z);
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    return (v.z() >= 0) ? (1/(2*M_PI)) : 0;
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    Point2f d = squareToUniformDisk(sample);
    float z = sqrt(max(0,1-d.x()*d.x()-d.y()*d.y()));
    return Vector3f(d.x(), d.y(), z);
    throw NoriException("Warp::squareToCosineHemisphere() is not yet implemented!");
}

double dotProduct(const Vector3f vectorA, const Vector3f vectorB) {
    double result = 0;
    for (size_t i = 0; i < vectorA.size(); ++i) {
        result += vectorA[i] * vectorB[i];
    }
    return result;
}

double vectorMagnitude(const Vector3f vector) {
    double sum = 0;
    sum = vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2];
    return sqrt(sum);
}

double angleBetweenVectors(const Vector3f vectorA, const Vector3f vectorB) {
    double dotProductValue = dotProduct(vectorA, vectorB);
    double magnitudeA = vectorMagnitude(vectorA);
    double magnitudeB = vectorMagnitude(vectorB);
    
    double cosineTheta = dotProductValue / (magnitudeA * magnitudeB);
    double thetaRad = acos(cosineTheta);
    return thetaRad;
}
float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    Vector3f northPole(0,0,1);
    float angle = angleBetweenVectors(v,northPole);
    if(angle <= 0){
        return 0;
    }

    return cos(angle) / M_PI;
    throw NoriException("Warp::squareToCosineHemispherePdf() is not yet implemented!");
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    throw NoriException("Warp::squareToBeckmann() is not yet implemented!");
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    throw NoriException("Warp::squareToBeckmannPdf() is not yet implemented!");
}

NORI_NAMESPACE_END
