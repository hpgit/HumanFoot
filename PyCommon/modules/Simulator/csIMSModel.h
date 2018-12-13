#ifndef _CSIMSMODEL_H_
#define _CSIMSMODEL_H_
#include "boostPythonUtil.h"

class Physics_ParticleSystem;
class Physics_SpringForce;

class ParticleConfig
{
public:
	ParticleConfig()
	{
		position = boost::python::make_tuple(0.,0.,0.);
		mass = 1.;
		initialVelocity = boost::python::make_tuple(0.,0.,0.);
		dynamicMu = 1.;
		staticMu = 1.;
	}
	ParticleConfig(const boost::python::object& position_, const double mass_, const boost::python::object& initialVelocity_, const double dynamicMu_, const double staticMu_)
		:position(position_), mass(mass_), initialVelocity(initialVelocity_), dynamicMu(dynamicMu_), staticMu(staticMu_)
	{}
	boost::python::object position;
	double mass;
	boost::python::object initialVelocity;
	double dynamicMu;
	double staticMu;
	string __str__();
};
struct ParticleConfig_pickle_suite : pickle_suite
{
	static boost::python::tuple getstate(const ParticleConfig& o)
	{
		return boost::python::make_tuple(o.position, o.mass, o.initialVelocity, o.dynamicMu, o.staticMu);
	}
	static void setstate(ParticleConfig& o, boost::python::tuple state)
	{
		o.position = state[0];
		o.mass = XD(state[1]);
		o.initialVelocity = state[2];
		o.dynamicMu = XD(state[3]);
		o.staticMu = XD(state[4]);
	}
};

class SpringConfig
{
public:
	SpringConfig(int particleIndex0_, int particleIndex1_, double Ks_, double Kd_)
		:particleIndex0(particleIndex0_), particleIndex1(particleIndex1_), Ks(Ks_), Kd(Kd_)
	{}
	int particleIndex0;
	int particleIndex1;
	double Ks;
	double Kd;
	string subspringsName;
	string __str__();
};
struct SpringConfig_pickle_suite : pickle_suite
{
	static boost::python::tuple getinitargs(SpringConfig& o)
	{
		return boost::python::make_tuple(o.particleIndex0, o.particleIndex1, o.Ks, o.Kd, o.subspringsName);
	}
	static boost::python::tuple getstate(const SpringConfig& o)
	{
		return boost::python::make_tuple(o.particleIndex0, o.particleIndex1, o.Ks, o.Kd, o.subspringsName);
	}
	static void setstate(SpringConfig& o, boost::python::tuple state)
	{
		o.particleIndex0 = XI(state[0]);
		o.particleIndex1 = XI(state[1]);
		o.Ks = XD(state[2]);
		o.Kd = XD(state[3]);
		o.subspringsName = XS(state [4]);
	}
};

class SystemConfig
{
public:
	SystemConfig()
	{
		g = boost::python::make_tuple(0.,-9.8,0.);
		tangentLockingVel = .01;
	}
	boost::python::object g;
	double tangentLockingVel;
	string __str__();
};
struct SystemConfig_pickle_suite : pickle_suite
{
	static boost::python::tuple getstate(const SystemConfig& o)
	{
		return boost::python::make_tuple(o.g, o.tangentLockingVel);
	}
	static void setstate(SystemConfig& o, boost::python::tuple state)
	{
		o.g = state[0] ;
		o.tangentLockingVel = XD(state[1]);
	}
};

// Implicit Mass Spring Model
class IMSModel
{
public:
	Physics_ParticleSystem* _pSystem;
	vector<Physics_SpringForce*> _springs;
	dict _subspringsMap;
public:
	void buildModel(const boost::python::list& particleConfigs, const boost::python::list& springConfigs);
public:	// expose to python
	IMSModel(const boost::python::list& particleConfigs,  const boost::python::list& springConfigs, const SystemConfig& systemConfig);
	void updateSprings(const boost::python::list& springLengths);
	void step(double timeStep);
	boost::python::tuple getPosition(int index);
	boost::python::tuple getVelocity(int index);
//	void setMu(const list& dynamicMuList, const list& staticMuList);
	void setMu(double dynamicMu, double staticMu, const boost::python::list& vertexIndices);
	boost::python::list getPositions();
	boost::python::list getVelocities();
	boost::python::list getContactParticleIndices();
// 	void setVelocity(int index, const object& velocity); 
	int getParticleNum();
	boost::python::list getState();
	void setState(const boost::python::object& state);
};

#endif // _CSIMSMODEL_H_