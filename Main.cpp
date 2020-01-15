#include <Siv3D.hpp> // OpenSiv3D v0.4.1
#include <vector>
#include <tuple>

#define X_OFFSET 50
#define Y_OFFSET 50
#define X_SCALE 50.0
#define Y_ACCURACY 100

int hasWater = 0;
double t = 0.0;  //時間
double t_0 = t; //水が噴出し終わった直後の時間
const double deg = 30; //発射角度
const double rad = deg * (s3d::Math::Pi / 180);
const double diameter = 93 * pow(10, -3);
const double A_0 = pow(diameter / 2, 2) * s3d::Math::Pi; //ペットボトル内の断面積
const double A = pow(8.45 * pow(10, -3) / 2, 2) * s3d::Math::Pi; //ペットボトルの口の断面積
double Pa = 101325.0; //大気圧
const double rho = 997.0; //水の密度
const double g = 9.8; //重力加速度
const double gamma = 1.4; //空気の比熱比
const double P_0 = 600000.0;  //初期のペットボトル内の気圧
const double R = 8.3; //気体定数
double T = 300; //温度
double T_1 = T; //時間で変化する噴出口近くの空気の温度
const double Cv = 5 / 2 * R; //定積モル比熱
const double Cd = 0.51; //抗力定数
const double rho_a = 1.293; //大気の中の空気の密度
const double m_0 = 0.0005; //初期の水の量
const double V_0 = 0.0015; //ペットボトルの体積
double m_1 = 0.0288; //1molあたりの空気の質量(kg)
double P = P_0;  //時間で変化しているボトル内部の気圧
double P_1 = P; //水が噴出し終わった直後の気圧
double P_2 = P; //計算する前の気圧(念のため)
const double bodyWeight = 0.1; //本体の質量
const double h_0 = (V_0 - m_0) / A_0; //初期の水面の高さ
double rho_0; //水が噴出し終わった直後の空気の密度
double rho_1 = rho_0; //時間で変化する空気の密度
double h = h_0; //時間で変化する水面の高さ
double u; //水が噴出する速度
double M = bodyWeight + rho * (V_0 / A_0 - h) * A_0; //時間で変化するペットボトル全体の質量
double F; //推力
double D = 0; //空気抵抗
double ax; //水平方向の加速度
double ay; //垂直方向の加速度
double vx_1;//水が噴出し終わった直後の水平方向の速度
double vx; //水平方向の速度
double vy_1;//水が噴出し終わった直後の垂直方向の速度
double vy; //垂直方向の速度
double x_1;//水が噴出し終わった直後の水平方向の距離
double x; //水平方向の距離
double y_1; //水が噴出し終わった直後の垂直方向の距離
double y; //垂直方向の距離
double mx = 0;//飛距離
const int windowX = 1000, windowY = 700;

std::vector<Vec2> pointsF;
std::vector<Vec2> pointsrho;
std::vector<Vec2> pointsP;
std::vector<Vec2> pointsT;
std::vector<Vec2> pointsM;
std::vector<Vec2> pointsH;
std::vector<Vec2> pointsD;
std::vector<Vec2> pointsAX;
std::vector<Vec2> pointsVX;
std::vector<Vec2> pointsAY;
std::vector<Vec2> pointsVY;
std::vector<Vec2> pointsX;
std::vector<Vec2> pointsY;

double Runge(double a0, double t0, double tn, double n ,int cw);
double Runge2(double a0, double b0, double t0, double tn, double n, int cw);
std::tuple<double,double> CoalitionRunge(double a0, double b0, double a, double b, double t0, double tn, double n, int cw1, int cw2);
void OrbitCalc();

void Main()
{
	const Font font(10);

	//画面のサイズの変更
	Window::Resize(windowX, windowY);

	while (System::Update())
	{
		OrbitCalc();
		//グラフの点の描画
		for (int i = 0; i < pointsF.size(); i++) {
			Circle(pointsF[i], 1).draw(Palette::Violet);
		}
		for (int i = 0; i < pointsAX.size(); i++) {
			Circle(pointsAX[i], 1).draw(Palette::Red);
		}
		for (int i = 0; i < pointsAY.size(); i++) {
			Circle(pointsAY[i], 1).draw(Palette::Darkred);
		}
		for (int i = 0; i < pointsVX.size(); i++) {
			Circle(pointsVX[i], 1).draw(Palette::Green);
		}
		for (int i = 0; i < pointsVY.size(); i++) {
			Circle(pointsVY[i], 1).draw(Palette::Darkgreen);
		}
		for (int i = 0; i < pointsX.size(); i++) {
			Circle(pointsX[i], 1).draw(Palette::Blue);
		}
		for (int i = 0; i < pointsY.size(); i++) {
			Circle(pointsY[i], 1).draw(Palette::Darkblue);
		}
		for (int i = 0; i < pointsP.size(); i++) {
			Circle(pointsP[i], 1).draw(Palette::Yellow);
		}
		for (int i = 0; i < pointsT.size(); i++) {
			Circle(pointsT[i], 1).draw(Palette::Orange);
		}
		for (int i = 0; i < pointsrho.size(); i++) {
			Circle(pointsrho[i], 1).draw(Palette::Pink);
		}
		for (int i = 0; i < pointsM.size(); i++) {
			Circle(pointsM[i], 1).draw(Palette::Brown);
		}
		for (int i = 0; i < pointsH.size(); i++) {
			Circle(pointsH[i], 1).draw(Palette::Skyblue);
		}
		for (int i = 0; i < pointsD.size(); i++) {
			Circle(pointsD[i], 1).draw(Palette::Gray);
		}
		for (int i = 0; i * 50 + X_OFFSET < windowX; i++) {
			font((double)(i * 5) / X_SCALE).draw(i * 50 + X_OFFSET, windowY - Y_OFFSET);
			Line(i * 50 + X_OFFSET, windowY - Y_OFFSET, i * 50 + X_OFFSET, windowY - Y_OFFSET + 5).draw();
			font(i * 50).draw(0, windowY - Y_OFFSET - i * 50);
			Line(X_OFFSET, windowY - Y_OFFSET - i * 50, X_OFFSET - 5, windowY - Y_OFFSET - i * 50).draw();
		}
		font(U"力").draw(windowX - 100, 0, Palette::Violet);
		font(U"加速度").draw(windowX - 100, 10, Palette::Red);
		font(U"速度").draw(windowX - 100, 20, Palette::Green);
		font(U"距離").draw(windowX - 100, 30, Palette::Blue);
		font(U"気圧").draw(windowX - 100, 40, Palette::Yellow);
		font(U"温度").draw(windowX - 100, 50, Palette::Orange);
		font(U"空気の密度").draw(windowX - 100, 60, Palette::Pink);
		font(U"ペットボトルの重さ").draw(windowX - 100, 70, Palette::Brown);
		font(U"ペットボトル内の水面の高さ").draw(windowX - 100, 80, Palette::Skyblue);
		font(U"抗力").draw(windowX - 100, 90, Palette::Gray);
		font(U"飛距離：").draw(windowX - 100, 100);
		font(mx).draw(windowX - 60, 100);
		//グラフの軸の描画
		Line(X_OFFSET, windowY - Y_OFFSET, X_OFFSET, 0).draw();
		Line(X_OFFSET, windowY - Y_OFFSET, windowX, windowY - Y_OFFSET).draw();
	}
}

//軌道の計算
void OrbitCalc() 
{
	t += (double)1 / X_SCALE;
	switch (hasWater) {
	case 0:
		//水がボトルに存在する時
		P = P_0 * pow(h_0 / h, gamma);
		u = sqrt(2 / rho * (P_0 * pow(h_0 / h, gamma) - Pa));
		rho_0 = m_1 * P / (R * T);
		rho_1 = rho_0;
		T = m_1 * P / (R * rho_1);
		M = bodyWeight + rho * (V_0 - h * A_0) + rho_1 * h * A_0;
		F = rho * A * pow(u, 2);
		h = Runge(h_0, 0, t, Y_ACCURACY, 1);
		ax = (F - D) / M * cos(rad);
		ay = (F - D) / M * sin(rad) - g;
		vx_1 = Runge(0, 0, t, Y_ACCURACY, 2);
		vy_1 = Runge(0, 0, t, Y_ACCURACY, 3);
		vx = vx_1;
		vy = vy_1;
		x_1 = Runge2(0, 0, 0, t, Y_ACCURACY, 2);
		y_1 = Runge2(0, 0, 0, t, Y_ACCURACY, 3);
		x = x_1;
		y = y_1;
		P_1 = P;
		t_0 = t;
		if (M <= bodyWeight + rho_0 * h * A_0) {
			hasWater++;
		}
		break;
	case 1:
		//水がボトルに存在しない時
		h = V_0 / A_0;
		T = m_1 * P / (rho_1 * R);
		M = bodyWeight + rho * (V_0 - h * A_0) + rho_1 * V_0;
		F = rho_1 * pow(Pa / P, 1 / gamma) * A * pow(u, 2);
		if (P > 1.89 * Pa) {
			u = sqrt(2 * gamma * P / ((gamma + 1) * rho_1));
			std::tie(rho_1, P) = CoalitionRunge(rho_0, P_1, rho_1, P, 0, t - t_0, Y_ACCURACY, 6, 7);
		}
		else {
			u = sqrt(2 * gamma * P / ((gamma - 1) * rho_1) * (1 - pow(Pa / P, (gamma - 1) / gamma)));
			std::tie(rho_1, P) = CoalitionRunge(rho_0, P_1, rho_1, P, 0, t - t_0, Y_ACCURACY, 4, 5);
		}
		ax = F / M * cos(rad);
		ay = F / M * sin(rad) - g;
		vx = Runge(vx_1, 0, t - t_0, Y_ACCURACY, 2);
		vy = Runge(vy_1, 0, t - t_0, Y_ACCURACY, 3);
		x = Runge2(x_1, vx_1, 0, t - t_0, Y_ACCURACY, 2);
		y = Runge2(y_1, vy_1, 0, t - t_0, Y_ACCURACY, 3);
		break;
	}
	if (y >= 0) {
		mx = x;
	}
	pointsF.emplace_back(Vec2{ t * X_SCALE + X_OFFSET,windowY - F * 10 - Y_OFFSET });
	pointsAX.emplace_back(Vec2{ t * X_SCALE + X_OFFSET,windowY - ax - Y_OFFSET });
	pointsVX.emplace_back(Vec2{ t * X_SCALE + X_OFFSET,windowY - vx - Y_OFFSET });
	pointsAY.emplace_back(Vec2{ t * X_SCALE + X_OFFSET,windowY - ay - Y_OFFSET });
	pointsVY.emplace_back(Vec2{ t * X_SCALE + X_OFFSET,windowY - vy - Y_OFFSET });
	pointsX.emplace_back(Vec2{ t * X_SCALE + X_OFFSET,windowY - x - Y_OFFSET });
	pointsY.emplace_back(Vec2{ t * X_SCALE + X_OFFSET,windowY - y - Y_OFFSET });
	pointsP.emplace_back(Vec2{ t * X_SCALE + X_OFFSET,windowY - P / 1000.0 - Y_OFFSET });
	pointsrho.emplace_back(Vec2{ t * X_SCALE + X_OFFSET,windowY - rho_1 * 10 - Y_OFFSET });
	pointsT.emplace_back(Vec2{ t * X_SCALE + X_OFFSET,windowY - T - Y_OFFSET });
	pointsM.emplace_back(Vec2{ t * X_SCALE + X_OFFSET,windowY - M * 1000 - Y_OFFSET });
	pointsH.emplace_back(Vec2{ t * X_SCALE + X_OFFSET,windowY - h * 1000 - Y_OFFSET });
	pointsD.emplace_back(Vec2{ t * X_SCALE + X_OFFSET,windowY - D - Y_OFFSET });
}

//微分方程式で解く式
double Calc(double a, double b, double ct, int cw)
{
	if (cw == 1) 
	{
		return A / A_0 * sqrt(2 / rho * (P_0 * pow(h_0 / b, gamma) - Pa));
	}
	else if (cw == 2) 
	{
		return ax - Cd * rho_a * pow(b, 2) * s3d::Math::Pi * diameter / 8;
	}
	else if (cw == 3)
	{
		return ay - Cd * rho_a * pow(b, 2) * s3d::Math::Pi * diameter / 8;
	}
	else if (cw == 4)
	{
		return -A / V_0 * a * pow(Pa / b, 1 / gamma) * sqrt(2 * gamma * b / ((gamma - 1) * a) * (1 - pow(Pa / b, (gamma - 1) / gamma)));
	}
	else if (cw == 5)
	{
		return -R * A * sqrt(2 * gamma * b / ((gamma - 1) * a) * (1 - pow(Pa / b, (gamma - 1) / gamma))) / (Cv * V_0) * (a * pow(Pa / b, 1 / gamma) / m_1 * Cv * T * (1 - (gamma - 1) * rho_1 * pow(u, 2) / (2 * gamma * b)) + a / 2 * pow(Pa / b, 1 / gamma) * pow(u, 2) + Pa);
	}
	else if (cw == 6)
	{
		return -A / V_0 * a * pow(2 / (gamma + 1), (gamma + 1) / (2 * (gamma - 1))) * sqrt(gamma * b / a);
	}
	else if (cw == 7)
	{
		return -R * A * sqrt(2 * gamma * b / ((gamma + 1) * a)) / (Cv * V_0) * (pow(2 / (gamma + 1), gamma / (gamma - 1)) * Cv * b / R + pow(2 / (gamma + 1), gamma / (gamma - 1)) * gamma * b / 2 + Pa);
	}
	else 
	{
		return (F - D) / M;
	}
}
//初期条件、初期時間、計算の終わり時間、計算の精度、どの式で計算か
double Runge(double a0, double t0, double tn, double n, int cw)
{
	double a, dt, ct, k1, k2, k3, k4;

	a = a0;
	ct = t0;
	dt = (tn - t0) / n;

	for (int i = 0; i <= n; i++) {
		ct = t0 + i * dt;
		k1 = dt * Calc(0, a, ct, cw);
		k2 = dt * Calc(0, a + k1 / 2, ct + dt / 2, cw);
		k3 = dt * Calc(0, a + k2 / 2, ct + dt / 2, cw);
		k4 = dt * Calc(0, a + k3, ct + dt, cw);
		a += (k1 + 2 * k2 + 2 * k3 + k4) / 6;
	}
	return a;
}

//初期条件、臨界条件、初期時間、計算の終わり時間、計算の精度、どの式で計算か
double Runge2(double a0, double b0, double t0, double tn, double n, int cw)
{
	double a, b, dt, ct, k1, k2, k3, k4, h1, h2, h3, h4;

	a = a0;
	b = b0;
	ct = t0;
	dt = (tn - t0) / n;

	for (int i = 0; i <= n; i++) {
		ct = t0 + i * dt;

		k1 = dt * b;
		h1 = dt * Calc(a, b, 0, cw);

		k2 = dt * (b + h1 / 2);
		h2 = dt * Calc(a + k1 / 2, b + h1 / 2, 0, cw);

		k3 = dt * (b + h2 / 2);
		h3 = dt * Calc(a + k2 / 2, b + h2 / 2, 0, cw);

		k4 = dt * (b + h3);
		h4 = dt * Calc(a + k3, b + h3, 0, cw);

		a += (k1 + 2 * k2 + 2 * k3 + k4) / 6;

		b += (h1 + 2 * h2 + 2 * h3 + h4) / 6; 
	}
	return a;
}

std::tuple<double,double> CoalitionRunge(double a0, double b0 ,double a, double b, double t0, double tn, double n, int cw1, int cw2) {
	double dt, ct, k1, k2, k3, k4, l1, l2, l3, l4;
	P_2 = b;//計算前の気圧を取得
	a = a0;
	b = b0;
	ct = t0;
	dt = (tn - t0) / n;
	for (int i = 0; i <= n; i++) {
		ct = t0 + i * dt;
		k1 = dt * Calc(a, b, ct, cw1);
		l1 = dt * Calc(a, b, ct, cw2);
		k2 = dt * Calc(a + k1 / 2, b + l1 / 2, ct + dt / 2, cw1);
		l2 = dt * Calc(a + k1 / 2, b + l1 / 2, ct + dt / 2, cw2);
		k3 = dt * Calc(a + k2 / 2, b + l2 / 2, ct + dt / 2, cw1);
		l3 = dt * Calc(a + k2 / 2, b + l2 / 2, ct + dt / 2, cw2);
		k4 = dt * Calc(a + k3, b + l3, ct + dt, cw1);
		l4 = dt * Calc(a + k3, b + l3, ct + dt, cw2);
		a += (k1 + 2 * k2 + 2 * k3 + k4) / 6;
		b += (l1 + 2 * l2 + 2 * l3 + l4) / 6;
	}
	if (isnan(b)) {
		return std::forward_as_tuple(m_1 * Pa / (R * T), Pa);
	}
	return std::forward_as_tuple(a, b);
}