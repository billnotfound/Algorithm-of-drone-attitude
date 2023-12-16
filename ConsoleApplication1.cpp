﻿// ConsoleApplication1.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
#include <iostream>
#include <math.h>
#include <time.h>
#define pi 3.141592653589793
#define SAMP_NUM samp_num
#define T 5//秒
#define samp 0.0025//弧度
#define float double
#define amp 0.01//弧度
#define phase 0//相位
#define samp_num 2000//采样点位
#define var_of_w 0.002//variance of angular velocity
#define var_of_g 0.0001//variance of angular gravity acceletation senser
#define drift 0.006//漂移
float pitch_sim[samp_num];//俯仰角
float white_noise[samp_num];//噪声
float w_withnoise_drift[samp_num];//输出角速度
float g_withnoise_y[samp_num];//
float g_withnoise_z[samp_num];
float g_withnoise_x[samp_num];//
float g_whitenoise[samp_num];
float a_with_noise_drift[samp_num];
float a_drift[SAMP_NUM];
float a_withoutnoise[samp_num];
float pitch;//x
float yaw;//z
float roll;//y
float pitch_cal_q0[samp_num];
float pitch_cal_q1[samp_num];
float pitch_cal_q2[samp_num];
float pitch_cal_q3[samp_num];
//欧拉角转四元数申明,输入xyz,输出q0,q1,q2,q3
void E2Q(float X, float Y, float Z, float& q0, float &q1, float &q2, float &q3)
{
	q0 = cos(Z / 2) * cos(Y / 2) * cos(X / 2) + sin(Z / 2) * sin(Y / 2) * sin(X / 2);
	q1 = sin(Z / 2) * cos(Y / 2) * cos(X / 2) - cos(Z / 2) * sin(Y / 2) * sin(X / 2);
	q2 = cos(Z / 2) * sin(Y / 2) * cos(X / 2) + sin(Z / 2) * cos(Y / 2) * sin(X / 2);
	q3 = cos(Z / 2) * cos(Y / 2) * sin(X / 2) - sin(Z / 2) * sin(Y / 2) * cos(X / 2);
}
//四元数转欧拉角输入q0,q1,q2,q3，输出xyz
void Q2E(float q0, float q1, float q2, float q3, float& X, float& Y, float& Z)
{
	X = atan2(2 * (q0 * q1 + q2 * q3), (1 - 2 * (q1 * q1 + q2 * q2)));
	Y = asin(2 * (q0 * q2 - q1 * q3));
	Z = atan2(2 * (q3 * q0 + q2 * q1), (1 - 2 * (q2 * q2 + q3 * q3)));
}
//四元数乘法，输入p，q；输出r
void multiplication(float p0, float p1, float p2, float p3, float q0, float q1, float q2, float q3, float& r0, float& r1, float& r2, float& r3)
{
	r0 = p0 * q0 - p1 * q1 - p2 * q2 - p3 * q3;
	r1 = p0 * q1 + p1 * q0 + p2 * q3 - p3 * q2;
	r2 = p0 * q2 - p1 * q3 + p2 * q0 + p3 * q1;
	r3 = p0 * q3 + p1 * q2 - p2 * q1 + p3 * q0;
}
//向量旋转，输入p，q，p0=0；输出p；输出p0仍为0
void rotate(float& p0, float &p1, float &p2, float &p3, float q0, float q1, float q2, float q3)
{
	float qp0, qp1, qp2, qp3;
	float qpq0, qpq1, qpq2, qpq3;
	multiplication(q0, q1, q2, q3, p0,p1,p2,p3,qp0, qp1, qp2, qp3);
	multiplication(qp0, qp1, qp2, qp3, q0, -q1, -q2, -q3, qpq0, qpq1, qpq2, qpq3);
	p0 = qpq0;
	p1 = qpq1;
	p2 = qpq2;
	p3 = qpq3;
}
//定义四元数结构
struct XX { float q0, q1, q2, q3; };
//四元数乘法，四元数×四元数,输入t,wx,wy,wz,q0,q1,q2,q3,返回q结构（q0q1q2q3）
XX XXX(float t, float wx, float wy, float wz,float q0,float q1,float q2,float q3) {
	XX q;

	q.q0 = q.q0 + t * (0 - wx * q.q1 - wy * q.q2 - wz * q.q3)/2;
	q.q1 = q.q1 + t * (0 + wx * q.q0 - wy * q.q3 + wz * q.q2)/2;
	q.q2 = q.q2 + t * (0 + wx * q.q3 + wy * q.q0 - wz * q.q1)/2;
	q.q3 = q.q3 + t * (0 - wx * q.q3 + wy * q.q1 + wz * q.q0)/2;
	return q;
}
struct V { float x, y, z; };//定义向量结构
//向量乘法，十字乘法输入x1x2y1y2z1z2,输出结构v，xyz
V mutiply(float x1, float y1, float z1, float x2, float y2, float z2) {
	V V;
	V.x = y1 * z2 - z1 * y2;
	V.y = z1 * y2 - x1 * z2;
	V.z = x1 * y2 - y1 * x2;
	return V;
}
void W2B(float q0, float q1, float q2, float q3, float &BX, float &BY, float &BZ)
{

	
	
	return;
}
//根据角速度更新四元数
void RK(float& q01, float& q11, float& q21, float& q31, float& q0, float& q1, float& q2,float& q3,float t,float wx,float wy,float wz) {
	q01 = q0 + 0.5 * t * (-wx * q1 - wy * q2 - wz * q3);
	q11 = q1 + 0.5 * t * (wx * q0 - wy * q3 + wz * q2);
	q21 = q2 + 0.5 * t * (wx * q3 + wy * q0 + wz * q1);
	q31 = q3 + 0.5 * t * (-wx * q2 + wy * q1 + wz * q0);
}
float WhiteNoise() {
	float a, b, x;
	a = ((float)rand())/ 32767;
	b = ((float)rand()) / 32767;
	x = sqrt((-2) * log(a)) * cos(2 * pi * b);
	return x;
}
float wave(int t) {

	return amp * sin(samp * t + phase);
}//正旋波动
float w(int t) {
	return amp * sin(samp * t + phase);
}
int main()
{
	int i, j;
	for (i = 0; i < 6; i++) {
		//printf();
	}
	for (i = 0; i < samp_num; i++) {
		white_noise[i] = WhiteNoise();
	}
	for (i = 0; i < samp_num; i++) {
		pitch_sim[i] = amp * sin(samp * i * 2 * pi + phase);
		w_withnoise_drift[i] = (amp * 2 * pi / T) * cos(2*pi*samp * i + phase) + drift + var_of_w * white_noise[i];
	}
	E2Q(0, 0, 0, pitch_cal_q0[0],pitch_cal_q1[0], pitch_cal_q2[0], pitch_cal_q3[0]);
	float Z, X, Y;
	for (i = 0;i < 1999;i++)
	{
		//printf("%f,%f\n", i * samp, wave(i));
		RK(pitch_cal_q0[i + 1], pitch_cal_q1[i + 1], pitch_cal_q2[i + 1], pitch_cal_q3[i + 1], pitch_cal_q0[i], pitch_cal_q1[i], pitch_cal_q2[i], pitch_cal_q3[i], samp, w_withnoise_drift[i], 0, 0);
		Q2E(pitch_cal_q0[i], pitch_cal_q1[i], pitch_cal_q2[i], pitch_cal_q3[i],X,Y,Z);
		a_with_noise_drift[i]=X;
		//printf("%f,%f,%f,%f\n", i * samp, X,Y,Z);
		//printf("%d\n",i);
	};//重力加速度传感器模拟（？）
	float a;
	for (i = 0; i < samp_num - 500; i++) {
		a = 0;
		for (j = i; j < i + 500; j++) {
			a = a + a_with_noise_drift[j];
		};
		a = a / 500;
		a_drift[i + 250] = a;
		a_withoutnoise[i + 250] + a_with_noise_drift[i + 250] + a_drift[i + 250];
		printf("%f,%f\n", i * samp+250, a_drift[i+250]);
	};
	for (i = 0; i < SAMP_NUM - 1; i++) {
		g_withnoise_x[i] = amp * sin(2 * pi * samp * i + phase) + var_of_g *32* white_noise[i];
		//printf("%f,%f\n", i * samp, g_withnoise_x[i]);
	};//这里没有问题。。。,信号源模拟（？）
	float g;
	for (i = 0; i < SAMP_NUM - 1; i++) {
		g = 0;
		for (j = i; j < i + 10; j++) {
			g = g + g_withnoise_x[j];
		};
		g = g / 10;
		//printf("%f,%f\n", i * samp, g);
	}//噪声去除。。。
	return 0;
}
