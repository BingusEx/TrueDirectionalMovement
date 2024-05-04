#pragma once
#include "Offsets.h"

#define PI 3.1415926535897932f
#define TWOTHIRDS_PI 2.0943951023931955f
#define TWO_PI 6.2831853071795865f
#define PI2 1.5707963267948966f
#define PI3 1.0471975511965977f
#define PI4 0.7853981633974483f
#define PI8 0.3926990816987242f

struct AngleZX
{
	double z;
	double x;
	double distance;
};

struct SoftPotential
{
	float k;
	float n;
	float s;
	float o;
	float a;
};

void GetAngle(const RE::NiPoint3& a_from, const RE::NiPoint3& a_to, AngleZX& angle);
bool GetAngle(RE::TESObjectREFR* a_target, AngleZX& angle);
RE::NiPoint3 GetCameraPos();
float NormalAbsoluteAngle(float a_angle);
float NormalRelativeAngle(float a_angle);
bool GetTorsoPos(RE::Actor* a_actor, RE::NiPoint3& point);
bool GetTargetPointPosition(RE::ObjectRefHandle a_target, std::string_view a_targetPoint, RE::NiPoint3& a_outPos);

void SetRotationMatrix(RE::NiMatrix3& a_matrix, float sacb, float cacb, float sb);
bool PredictAimProjectile(RE::NiPoint3 a_projectilePos, RE::NiPoint3 a_targetPosition, RE::NiPoint3 a_targetVelocity, float a_gravity, RE::NiPoint3& a_projectileVelocity);

[[nodiscard]] inline float GetPlayerTimeMultiplier()
{
	return GetPlayerTimeMult(*g_142EC5C60);
}

[[nodiscard]] inline float GetPlayerDeltaTime()
{
    return *g_deltaTime * GetPlayerTimeMultiplier();
}

[[nodiscard]] inline float GetRealTimeDeltaTime()
{
    return *g_deltaTimeRealTime;
}

[[nodiscard]] inline RE::NiPoint3 TransformVectorByMatrix(const RE::NiPoint3& a_vector, const RE::NiMatrix3& a_matrix)
{
	return RE::NiPoint3(a_matrix.entry[0][0] * a_vector.x + a_matrix.entry[0][1] * a_vector.y + a_matrix.entry[0][2] * a_vector.z,
		a_matrix.entry[1][0] * a_vector.x + a_matrix.entry[1][1] * a_vector.y + a_matrix.entry[1][2] * a_vector.z,
		a_matrix.entry[2][0] * a_vector.x + a_matrix.entry[2][1] * a_vector.y + a_matrix.entry[2][2] * a_vector.z);
}

[[nodiscard]] inline float AngleToRadian(float a_angle)
{
	return a_angle * 0.017453292f;
}

[[nodiscard]] inline float RadianToAngle(float a_radian)
{
	return a_radian * 57.295779513f;
}

[[nodiscard]] inline bool ApproximatelyEqual(float A, float B)
{
	return ((A - B) < FLT_EPSILON) && ((B - A) < FLT_EPSILON);
}

[[nodiscard]] inline RE::NiPoint2 Vec2Rotate(const RE::NiPoint2& vec, float angle)
{
	RE::NiPoint2 ret;
	ret.x = vec.x * cos(angle) - vec.y * sin(angle);
	ret.y = vec.x * sin(angle) + vec.y * cos(angle);
	return ret;
}

[[nodiscard]] inline RE::NiPoint3 RotateAngleAxis(const RE::NiPoint3& vec, const float angle, const RE::NiPoint3& axis)
{
	float S = sin(angle);
	float C = cos(angle);

	const float XX = axis.x * axis.x;
	const float YY = axis.y * axis.y;
	const float ZZ = axis.z * axis.z;

	const float XY = axis.x * axis.y;
	const float YZ = axis.y * axis.z;
	const float ZX = axis.z * axis.x;

	const float XS = axis.x * S;
	const float YS = axis.y * S;
	const float ZS = axis.z * S;

	const float OMC = 1.f - C;

	return RE::NiPoint3(
		(OMC * XX + C) * vec.x + (OMC * XY - ZS) * vec.y + (OMC * ZX + YS) * vec.z,
		(OMC * XY + ZS) * vec.x + (OMC * YY + C) * vec.y + (OMC * YZ - XS) * vec.z,
		(OMC * ZX - YS) * vec.x + (OMC * YZ + XS) * vec.y + (OMC * ZZ + C) * vec.z
	);
}

[[nodiscard]] inline RE::NiPoint3 RotateVector(const RE::NiPoint3& a_vec, const RE::NiQuaternion& a_quat)
{
	//http://people.csail.mit.edu/bkph/articles/Quaternions.pdf
	const RE::NiPoint3 Q{ a_quat.x, a_quat.y, a_quat.z };
	const RE::NiPoint3 T = Q.Cross(a_vec) * 2.f;
	return a_vec + (T * a_quat.w) + Q.Cross(T);
}

[[nodiscard]] inline RE::NiPoint3 ClampSizeMax(const RE::NiPoint3& vec, const float max)
{
	if (max < 1.e-4f)
	{
		return RE::NiPoint3 {0, 0, 0};
	}

	const float squaredLength = vec.SqrLength();
	if (squaredLength > max * max) {
		const float scale = max * (1.0f / std::sqrt(squaredLength));
		return vec * scale;
	} else {
		return vec;
	}
}

//inline float ClampAngle(float angle, float min, float max)
//{
//	return fmod(angle, max - min) + min;
//}

[[nodiscard]] inline float ClipAngle(float angle, float min, float max)
{
	return fmin(max, fmax(min, angle));
}

[[nodiscard]] inline float GetAngle(RE::NiPoint2& a, RE::NiPoint2& b)
{
	return atan2(a.Cross(b), a.Dot(b));
}

[[nodiscard]] inline RE::NiPoint3 ToOrientationRotation(const RE::NiPoint3& a_vector)
{
	RE::NiPoint3 ret;

	// Pitch
	ret.x = atan2(a_vector.z, std::sqrtf(a_vector.x * a_vector.x + a_vector.y * a_vector.y));

	// Roll
	ret.y = 0;

	// Yaw
	ret.z = atan2(a_vector.y, a_vector.x);

	return ret;
}

[[nodiscard]] inline RE::NiPoint3 RotationToDirection(const float a_yaw, const float a_pitch)
{
	RE::NiPoint3 ret;

	float CP, SP, CY, SY;
	CP = cos(a_pitch);
	SP = sin(a_pitch);
	CY = cos(a_yaw);
	SY = sin(a_yaw);

	ret.x = CP * CY;
	ret.y = CP * SY;
	ret.z = SP;

	return ret;
}

[[nodiscard]] inline RE::NiPoint3 Project(const RE::NiPoint3& A, const RE::NiPoint3& B)
{
	return (B * ((A.x * B.x + A.y * B.y + A.z * B.z) / (B.x * B.x + B.y * B.y + B.z * B.z)));
}

[[nodiscard]] inline float Clamp(float value, float min, float max)
{
	return value < min ? min : value < max ? value : max;
}

[[nodiscard]] inline float InterpEaseIn(const float& A, const float& B, float alpha, float exp)
{
	float const modifiedAlpha = std::pow(alpha, exp);
	return std::lerp(A, B, modifiedAlpha);
}

[[nodiscard]] inline float InterpEaseOut(const float& A, const float& B, float alpha, float exp)
{
	float const modifiedAlpha = 1.f - pow(1.f - alpha, exp);
	return std::lerp(A, B, modifiedAlpha);
}

[[nodiscard]] inline float InterpEaseInOut(const float& A, const float& B, float alpha, float exp)
{
	return std::lerp(A, B, (alpha < 0.5f) ? InterpEaseIn(0.f, 1.f, alpha * 2.f, exp) * 0.5f : InterpEaseOut(0.f, 1.f, alpha * 2.f - 1.f, exp) * 0.5f + 0.5f);
}

[[nodiscard]] inline float InterpTo(float a_current, float a_target, float a_deltaTime, float a_interpSpeed)
{
	if (a_interpSpeed <= 0.f) {
		return a_target;
	}

	const float distance = a_target - a_current;

	if (distance * distance < FLT_EPSILON) {
		return a_target;
	}

	const float delta = distance * Clamp(a_deltaTime * a_interpSpeed, 0.f, 1.f);

	return a_current + delta;
}

[[nodiscard]] inline float InterpAngleTo(float a_current, float a_target, float a_deltaTime, float a_interpSpeed)
{
	if (a_interpSpeed <= 0.f) {
		return a_target;
	}

	const float distance = NormalRelativeAngle(a_target - a_current);

	if (distance * distance < FLT_EPSILON) {
		return a_target;
	}

	const float delta = distance * Clamp(a_deltaTime * a_interpSpeed, 0.f, 1.f);

	return a_current + delta;
}

[[nodiscard]] inline float GetAngleDiff(const float& A, const float& B)
{
	return PI - fabs(fmod(fabs(A - B), TWO_PI) - PI);
}

[[nodiscard]] inline bool FloatCompare(const float a, const float b)
{
	double delta = fabs(a - b);
	if (delta < std::numeric_limits<float>::epsilon() &&
		delta > -std::numeric_limits<float>::epsilon()) {
		return true;
	}
	return false;
}

[[nodiscard]] inline float GetPct(const float a_current, const float a_max)
{
	float percent = -1.f;

	if (a_max < 0.f) {
		return percent;
	}

	if (!FloatCompare(a_max, 0.f)) {
		//percent = ceil((a_current / a_max) * 100.f);
		percent = a_current / a_max;
		//return fmin(100.f, fmax(percent, -1.f));  // negative indicates that the actor value is not used
		return fmin(1.f, fmax(percent, -1.f));  // negative indicates that the actor value is not used
	}

	return percent;
}

//TODO All of this should really be an API call into th gts dll..

[[nodiscard]] RE::hkVector4 GetBoneQuad(const RE::Actor* a_actor, const char* a_boneStr, bool a_invert, bool a_worldtranslate);

[[nodiscard]] RE::NiAVObject* FindBoneNode(const RE::Actor* a_actorptr, std::string_view a_nodeName, bool a_isFirstPerson);

[[nodiscard]] inline float soft_power(const float x, const float k, const float n, const float s, const float o, const float a)
{
	return pow(1.0f + pow(k * (x), n * s), 1.0f / s) / pow(1.0f + pow(k * o, n * s), 1.0f / s) + a;
}

[[nodiscard]] inline float soft_core(const float x, const float k, const float n, const float s, const float o, const float a)
{
	return 1.0f / soft_power(x, k, n, s, o, 0.0) + a;
}

[[nodiscard]] inline float soft_core(const float x, const SoftPotential& soft_potential)
{
	return soft_core(x, soft_potential.k, soft_potential.n, soft_potential.s, soft_potential.o, soft_potential.a);
}

[[nodiscard]] inline RE::NiPoint3 GetNiPoint3(RE::hkVector4 a_hkVector4)
{
	float quad[4];
	_mm_store_ps(quad, a_hkVector4.quad);
	return RE::NiPoint3{ quad[0], quad[1], quad[2] };
}

[[nodiscard]] inline float Remap(const float a_oldValue, const float a_oldMin, const float a_oldMax, const float a_newMin, const float a_newMax)
{
	return (((a_oldValue - a_oldMin) * (a_newMax - a_newMin)) / (a_oldMax - a_oldMin)) + a_newMin;
}

[[nodiscard]] inline float GetRefScale(RE::Actor* actor)
{
	// This function reports same values as GetScale() in the console, so it is a value from SetScale() command
	return static_cast<float>(actor->GetReferenceRuntimeData().refScale) / 100.0F;
}

[[nodiscard]] inline float GetModelScale(const RE::Actor* a_actor)
{
	if (!a_actor->Is3DLoaded()) {
		return 1.0;
	}

	if (const auto model = a_actor->Get3D(false)) {
		return model->local.scale;
	}

	if (const auto first_model = a_actor->Get3D(true)) {
		return first_model->local.scale;
	}

	return 1.0;
}

[[nodiscard]] inline float GetNodeScale(const RE::Actor* a_actor, const std::string_view a_boneName)
{
	if (const auto Node = FindBoneNode(a_actor, a_boneName, false)) {
		return Node->local.scale;
	}
	if (const auto FPNode = FindBoneNode(a_actor, a_boneName, true)) {
		return FPNode->local.scale;
	}
	return -1.0;
}

[[nodiscard]] inline RE::hkVector4 NiPointToHkVector(const RE::NiPoint3& a_point, bool a_convertScale = false)
{
	RE::hkVector4 ret = { a_point.x, a_point.y, a_point.z, 0 };
	if (a_convertScale) {
		ret = ret * *g_worldScale;
		logger::trace("(NiPointToHKVector) Worldscale: {} Resulting Output: X:{}, Y:{}, Z:{},", *g_worldScale, ret.quad.m128_f32[0], ret.quad.m128_f32[1], ret.quad.m128_f32[2]);
	}
	return ret;
}

[[nodiscard]] inline float GetScale(const RE::Actor* a_actor)
{
	float TargetScale = 1.f;
	TargetScale *= GetModelScale(a_actor);                    //Model scale, Scaling done by game
	TargetScale *= GetNodeScale(a_actor, "NPC");              // NPC bone, Racemenu uses this.
	TargetScale *= GetNodeScale(a_actor, "NPC Root [Root]");  //Child bone of "NPC" some other mods scale this bone instead

	if (TargetScale < 0.15f)
		TargetScale = 0.15f;
	if (TargetScale > 250.f)
		TargetScale = 250.f;
	return TargetScale;
}

[[nodiscard]] inline float GetAnimationSlowdown(RE::Actor* a_actor)
{
	if (!a_actor)
		return 1.0;

	SoftPotential getspeed{
		.k = 0.142f,  // 0.125
		.n = 0.82f,   // 0.86
		.s = 1.90f,   // 1.12
		.o = 1.0f,
		.a = 0.0f,  //Default is 0
	};

	return static_cast<float>(pow(soft_core(GetScale(a_actor), getspeed), 1.2));
	//return static_cast<float>(soft_core(GetScale(a_actor), getspeed));
}
