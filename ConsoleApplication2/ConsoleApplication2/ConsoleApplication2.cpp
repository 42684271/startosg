﻿// ConsoleApplication2.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "pch.h"
#include <iostream>

#include <osg/Node>
#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
#include <osg/NodeVisitor>
#include <osg/StateSet>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Drawable>
#include <osgDb/WriteFile>
#include <osg/BoundingSphere>
#include <osg/Point>

#include <iostream>
#include <ctime>

#define random(x)			(rand()%x)
#define random_range(a, b)	((rand() % (b - a)) + a)

////////////////////////////////////////////////////////////////////////////////////////////////////
class MeteorCB : public osg::NodeCallback
{
public:
	MeteorCB(unsigned int lifetime, osg::Vec3 velocity)
		: _lifetime(lifetime), _velocity(velocity)
	{
		_vIncrease = osg::PI_4 / (_lifetime/100. );
		_a = 0.;
		_v = osg::Vec3(0.,0.,0.);
	
	};

	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		osg::MatrixTransform* mt = dynamic_cast<osg::MatrixTransform*> (node);

		//osg::Vec3 translation;
		//osg::Quat rotation;
		//osg::Vec3 scale;
		//osg::Quat so;

		osg::Matrix m;

		_a += _vIncrease;
		_v = _velocity * sin(osg::PI_4 + _a) * 50;
		//_velocity += _velocity * sin(osg::PI_4 + _a);
		m.makeTranslate(_velocity + _v);

		if ( _lifetime-- > 0) mt->postMult(m);

		traverse(node, nv);
	}


private:
	unsigned int _lifetime;
	osg::Vec3 _velocity;

	double _vIncrease;
	double _a;
	osg::Vec3 _v;


};



class Meteor : public osg::MatrixTransform
{
public:
	Meteor() : _velocity(osg::Vec3(10., 10., 10.))
		, _initialPosition(osg::Vec3(0., 0., 0))
		, _lifetime(1024)
		, _headsize(2.)
		, _bodysize(10.) {};

	Meteor(osg::Vec3 velocity, osg::Vec3 position
		, unsigned int lifetime
		, float headsize
		, float bodysize)
	{
		_velocity = velocity;
		_initialPosition = position;
		_lifetime = lifetime;
		_headsize = headsize;
		_bodysize = bodysize;

		osg::ref_ptr < osg::Geode > n = new osg::Geode;
		osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;

		osg::ref_ptr<osg::Vec3Array> vhead = new osg::Vec3Array;
		osg::ref_ptr<osg::Vec3Array> vbody = new osg::Vec3Array;
		osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array;
		osg::ref_ptr<osg::Vec3Array> norm = new osg::Vec3Array;

		osg::Vec3 pose = _velocity;
		pose.normalize();

		//vhead->push_back(position);

		vbody->push_back(position);
		vbody->push_back(osg::Vec3(position.x() + pose.x()* bodysize
			, position.y() + pose.x() * bodysize
			, position.z() + pose.z() * bodysize
			)
		);

		geom->setVertexArray(vbody.get());
		c->push_back(osg::Vec4(1., 1., 1., 1.));
		geom->setColorArray(c.get());
		geom->setColorBinding(osg::Geometry::BIND_OVERALL);

		//geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, 1));
		geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2));

		osg::StateSet* stateset = new osg::StateSet;
		osg::LineWidth* linewidth = new osg::LineWidth();
		linewidth->setWidth(headsize);
		stateset->setAttributeAndModes(linewidth, osg::StateAttribute::ON);
		stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
		geom->setStateSet(stateset);

		setUpdateCallback(new MeteorCB(_lifetime, _velocity));

		n->addDrawable(geom.get());
		addChild(n);

	}

	const osg::Vec3 getVeloticy() const { return _velocity; }
	const osg::Vec3 getInitialPosition() const { return _initialPosition; }
	unsigned int getLifeTime() const { return _lifetime; }

private:

	osg::Vec3 _velocity;
	osg::Vec3 _initialPosition;
	unsigned int _lifetime;
	float _headsize;
	float _bodysize;


};


////////////////////////////////////////////////////////////////////////////////////////////////////
const float star_size_big		= 3.;
const float star_size_medium	= 2.;
const float star_size_small		= 1;
const int star_layer_width	= 1000000;

osg::ref_ptr<osg::Node> createStar(double dBase
	, unsigned int uCount
	, osg::Vec4Array* colors
	, float fStarSize)
{
	osg::ref_ptr<osg::Geode> stars = new osg::Geode;
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;

	osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array;
	osg::ref_ptr<osg::Vec3Array> n = new osg::Vec3Array;

	geom->setVertexArray(v.get());
	geom->setColorArray(c.get());

	//double alpha = 0., beta = 0.;
	double alpha = 0., beta = 0., thelta = 0.;
	double r = dBase + star_layer_width;

	srand(time(NULL));
	for (unsigned int b = 0; b < uCount; b++)
	{
#define N 9999
		alpha	= rand() % (N + 1) / (float)(N + 1) * osg::PI - osg::PI_2;
		beta	= rand() % (N + 1) / (float)(N + 1) * osg::PI * 2. - osg::PI;
		double x = r * cos(alpha) * cos(beta);
		double y = r * cos(alpha) * sin(beta);
		double z = r * sin(alpha);

		osg::Vec3 norm = osg::Vec3(x, y, z);
		norm.normalize();

		//float offset = (float)(star_layer_width) * ( rand() % (N + 1) / (float)(N + 1) );
		float offset = (float)(dBase) * (rand() % (N + 1) / (float)(N + 1));

		x += norm.x() * offset ;
		y += norm.y() * offset;
		z += norm.z() * offset;

		v->push_back(osg::Vec3(x, y, z));

		n->push_back(-norm);
		c->push_back((*colors)[random_range(0, colors->size())]);
		//c->push_back(osg::Vec4(1.,1.,1.,1.));
	}
	
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, uCount));
	geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	geom->setNormalArray(n);
	geom->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

	osg::StateSet* stateset = geom->getOrCreateStateSet();
	osg::Point* point = new osg::Point;
	point->setSize(fStarSize);
	stateset->setAttributeAndModes(point, osg::StateAttribute::ON);

	stars->addDrawable(geom);

	return stars.get();

}


class PlanetCB : public osg::NodeCallback
{
public:
	PlanetCB() : _angularSpeedRot(0.), _angularSpeedRevo(0.)
		, _orbitRadius(osg::Vec3(100., 0., 0))
		, _equatorRadius(osg::Vec3(1., 1., 1.)), _rot(0.), _revo(0.) {
	};

	PlanetCB(double rot, double revo, osg::Vec3 orbit, osg::Vec3 equator)
	{
		_angularSpeedRot = rot;
		_angularSpeedRevo = revo;
		_orbitRadius = orbit;
		_equatorRadius = equator;
		_rot = 0.0;
		_revo = 0.;
	}

	//~PlanetCB();

	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{

		osg::MatrixTransform* mt = dynamic_cast<osg::MatrixTransform*> (node);
		
		osg::Matrix m;

		osg::Matrix mS, mT, mRot, mReov;
		mS.makeScale(_equatorRadius);
		mRot.makeRotate(_rot, osg::Vec3(0., 1.0, 0.));
		mT.makeTranslate(_orbitRadius);
		mReov.makeRotate(_revo, osg::Vec3(0., 1., 0.));
		//mRot.makeRotate();

		mt->setMatrix(mS * mRot * mT * mReov);

		_rot  += _angularSpeedRot;
		_revo += _angularSpeedRevo;

		traverse(node, nv);
	}

private:
	osg::Vec3 _orbitRadius;
	osg::Vec3 _equatorRadius;
	double _angularSpeedRot;
	double _angularSpeedRevo;

	double _rot;
	double _revo;
};


// a typical planet could renovate, rotate and possibly has one or more satellite,
// which could be a planet also.

class Planet : public osg::MatrixTransform
{
public:
	Planet() : _angularSpeedRot(0.), _angularSpeedRevo(0.), _orbitRadius(osg::Vec3(100., 0., 0.)),
		_equatorRadius(osg::Vec3(1., 1., 1.)) { };

	Planet(double _angularSpeedRot, double _angularSpeedRevo, osg::Vec3 orbitRadius, osg::Vec3 equator)
	{
		_orbitRadius = orbitRadius;
		_equatorRadius = equator;
		setUpdateCallback(new PlanetCB(0., _angularSpeedRevo, _orbitRadius, osg::Vec3(1.,1.,1.)));
		osg::StateSet* state = getOrCreateStateSet();
		state->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);

		_move = new osg::MatrixTransform;
		_move->setUpdateCallback(new PlanetCB(_angularSpeedRot, 0, osg::Vec3(0., 0., 0.), _equatorRadius));
		state = getOrCreateStateSet();
		state->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);

		addChild(_move);
	}

	virtual void addSubStar(osg::Node* star) {	addChild(star);	}
	virtual void setSphere(osg::Node* sphere) { _move->addChild(sphere);  }
	virtual void makeMovement(osg::NodeCallback* cb) { setUpdateCallback(cb);  }

	//~Planet();

	enum OrbitLevel
	{
		ORBIT_SIMPLE= 0,
		ORBIT_DETAIL= 1,
		ORBIT_DENSE = 2
	} ;

	void showOrbit(bool bVisible=true, osg::Vec4 color=osg::Vec4(1., .5, 0., 1.), Planet::OrbitLevel ol= ORBIT_SIMPLE )
	{
		_orbitLevel = ol;
		if (!_orbit.valid() && bVisible)
		{
			_orbit = new osg::Geode;
			osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;

			osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array;
			unsigned int orbitL = 360;
			switch (_orbitLevel)
			{
			case Planet::ORBIT_SIMPLE:
				orbitL = 360;
				break;
			case Planet::ORBIT_DETAIL:
				orbitL = 540;
				break;
			case Planet::ORBIT_DENSE:
				orbitL = 720;
				break;
			default:
				break;
			}

			for (unsigned i = 0; i < orbitL; i++)
			{
				v->push_back(osg::Vec3(_orbitRadius.length() * cos(osg::PI * 2 *i / orbitL)
					, 0.
					, _orbitRadius.length() * sin(osg::PI * 2 * i / orbitL)) - _orbitRadius);
			}
			geom->setVertexArray(v.get());

			// color
			osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array;
			c->push_back(color);
			geom->setColorArray(c.get());
			geom->setColorBinding(osg::Geometry::BIND_OVERALL);

			// normal
			osg::ref_ptr<osg::Vec3Array> n = new osg::Vec3Array;
			n->push_back(osg::Vec3(0,1,0));
			geom->setNormalArray(n.get());
			geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP, 0, orbitL));
			
			_orbit->addDrawable(geom);

			// stateset
			osg::StateSet* stateset = new osg::StateSet;
			osg::LineWidth* linewidth = new osg::LineWidth();
			linewidth->setWidth(.5f);
			stateset->setAttributeAndModes(linewidth, osg::StateAttribute::ON);
			stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
			geom->setStateSet(stateset);

			addChild(_orbit);

			return;
		}

		removeChild(_orbit);

	}
	

private:
	osg::ref_ptr<osg::MatrixTransform>	_move;
	osg::ref_ptr<osg::Geode>			_orbit;
	OrbitLevel _orbitLevel;

	double _angularSpeedRot;
	double _angularSpeedRevo;

	osg::Vec3	_orbitRadius;
	osg::Vec3	_equatorRadius;
};


osg::ref_ptr<osg::Node> createSceneGraph1()
{

	//osg::ref_ptr<osg::Node> model = osgDB::readNodeFile("sphere-ems-yellow.3ds");
	//osg::ref_ptr<osg::Node> model = osgDB::readNodeFile("sphere-material.3ds");
	osg::ref_ptr<osg::Node> model_sun = osgDB::readNodeFile("sphere-material-2.3ds");
	model_sun->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
	osg::ref_ptr<osg::Node> model_mercury = osgDB::readNodeFile("sphere-material-mercury.3ds");
	model_mercury->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
	osg::ref_ptr<osg::Node> model_venus = osgDB::readNodeFile("sphere-material-venus.3ds");
	model_venus->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
	osg::ref_ptr<osg::Node> model_earth = osgDB::readNodeFile("sphere-material-earth.3ds");
	model_earth->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
	osg::ref_ptr<osg::Node> model_mars = osgDB::readNodeFile("sphere-material-mars.3ds");
	model_mars->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
	osg::ref_ptr<osg::Node> model_jupiter = osgDB::readNodeFile("sphere-material-jupiter.3ds");
	model_jupiter->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
	osg::ref_ptr<osg::Node> model_saturn = osgDB::readNodeFile("sphere-material-saturn.3ds");
	model_saturn->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
	osg::ref_ptr<osg::Node> model_uranus = osgDB::readNodeFile("sphere-material-uranus.3ds");
	model_uranus->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
	osg::ref_ptr<osg::Node> model_neptune = osgDB::readNodeFile("sphere-material-neptune.3ds");
	model_neptune->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
	osg::ref_ptr<osg::Node> model_moon = osgDB::readNodeFile("sphere-material-moon.3ds");
	model_moon->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);

// 	if (!model.valid())
// 	{
// 		osg::notify(osg::FATAL) << "Open model file error!" << std::endl;
// 		exit(1);
// 	}

	//factor - to adapt to frame update.
	double dOrbitRadiusFactor	= 200.0;
	double dEquatorRadiusFactor	= 10.0;
	double dRotFactor			= 0.0005;
	double dRevoFactor			= 0.000005;

	double sun_orbitRadius	= 0.00 *dOrbitRadiusFactor;
	double mercury_orbitRadius	= 150.	*dOrbitRadiusFactor;
	double venus_orbitRadius	= 281.	*dOrbitRadiusFactor;
	double earth_orbitRadius	= 395.	*dOrbitRadiusFactor;
	double mars_orbitRadius		= 648.	*dOrbitRadiusFactor;
	double jupiter_orbitRadius	= 1926.	*dOrbitRadiusFactor;
	double saturn_orbitRadius	= 3936.	*dOrbitRadiusFactor;
	double uranus_orbitRadius	= 7151.	*dOrbitRadiusFactor;
	double neptune_orbitRadius	= 11716.*dOrbitRadiusFactor;
	//double moon_orbitRadius		= 1.0	*dOrbitRadiusFactor;
	double moon_orbitRadius = 20.0	*dOrbitRadiusFactor;

	//double sun_radius = 285.24 *dEquatorRadiusFactor;
	double sun_radius = 285.24 *dEquatorRadiusFactor *0.2;
	double mercury_radius	= 1.00 *dEquatorRadiusFactor;
	double venus_radius		= 2.47 *dEquatorRadiusFactor;
	double earth_radius		= 2.58 *dEquatorRadiusFactor;
	double mars_radius		= 1.39 *dEquatorRadiusFactor;
	double jupiter_radius	= 28.65*dEquatorRadiusFactor;
	double saturn_radius	= 24.70*dEquatorRadiusFactor;
	double uranus_radius	= 10.23*dEquatorRadiusFactor;
	double neptune_radius	= 10.14*dEquatorRadiusFactor;
	double moon_radius		= 0.7123*dEquatorRadiusFactor;

	double sun_rotation = 12.12 *dRotFactor;
	double mercury_rotation		= 4.14	*dRotFactor;
	double venus_rotation		= 1.00	*dRotFactor;
	double earth_rotation		= 242.99*dRotFactor;
	double mars_rotation		= 236.85*dRotFactor;
	double jupiter_rotation		= 595.10*dRotFactor;
	double saturn_rotation		= 555.42*dRotFactor;
	double uranus_rotation		= 337.10*dRotFactor;
	double neptune_rotation		= 364.49*dRotFactor;
	double moon_rotation		= 8.89  *dRotFactor;

	double sun_revolution = 0.0 *dRevoFactor;
	double mercury_revolution	= 685.85 *dRevoFactor;
	double venus_revolution		= 268.48 *dRevoFactor;
	double earth_revolution		= 165.28 *dRevoFactor;
	double mars_revolution		= 87.81  *dRevoFactor;
	double jupiter_revolution	= 13.93  *dRevoFactor;
	double saturn_revolution	= 5.56   *dRevoFactor;
	double uranus_revolution	= 1.96   *dRevoFactor;
	double neptune_revolution	= 1.00   *dRevoFactor;
	double moon_revolution		= 92.00  *dRevoFactor;


	// planet model
	//osg::StateSet* state = model->getOrCreateStateSet();
	//state->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);
	//state->setMode(GL_NORMALIZE, osg::StateAttribute::ON);

	// sun
	//osg::ref_ptr<Planet> sun = new Planet(0., 0., osg::Vec3(0.,0.,0.), osg::Vec3(1., 1., 1.));
	osg::ref_ptr<Planet> sun = new Planet(sun_rotation, sun_revolution
		, osg::Vec3(sun_orbitRadius, 0., 0.)
		, osg::Vec3(sun_radius, sun_radius, sun_radius));
	sun->setSphere(model_sun);
	//osg::StateSet* sunStates = sun->getOrCreateStateSet();
	//sunStates->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	// mercury
	osg::ref_ptr<Planet> mercury = new Planet(mercury_rotation, mercury_revolution
		, osg::Vec3(mercury_orbitRadius, 0., 0.)
		, osg::Vec3(mercury_radius, mercury_radius, mercury_radius));
	mercury->setSphere(model_mercury);
	mercury->showOrbit(true);
	sun->addSubStar(mercury);


	// venus
	osg::ref_ptr<Planet> venus = new Planet(venus_rotation, venus_revolution
		, osg::Vec3(venus_orbitRadius, 0., 0.)
		, osg::Vec3(venus_radius, venus_radius, venus_radius));
	venus->setSphere(model_venus);
	venus->showOrbit(true);
	sun->addSubStar(venus);


	osg::ref_ptr<Planet> earth = new Planet(earth_rotation, earth_revolution
		, osg::Vec3(earth_orbitRadius, 0., 0.)
		, osg::Vec3(earth_radius, earth_radius, earth_radius));
	earth->setSphere(model_earth);
	earth->showOrbit(true);
	sun->addSubStar(earth);

	{
		// moon
		osg::ref_ptr<Planet> moon = new Planet(moon_rotation, moon_revolution
			, osg::Vec3(moon_orbitRadius, 0., 0.)
			, osg::Vec3(moon_radius, moon_radius, moon_radius));
		moon->setSphere(model_moon);
		moon->showOrbit(true);
		earth->addSubStar(moon);

	}

	// mars
	osg::ref_ptr<Planet> mars = new Planet(mars_rotation, mars_revolution
		, osg::Vec3(mars_orbitRadius, 0., 0.)
		, osg::Vec3(mars_radius, mars_radius, mars_radius));
	mars->setSphere(model_mars);
	mars->showOrbit(true);
	sun->addSubStar(mars);


	// jupiter
	osg::ref_ptr<Planet> jupiter = new Planet(jupiter_rotation, jupiter_revolution
		, osg::Vec3(jupiter_orbitRadius, 0., 0.)
		, osg::Vec3(jupiter_radius, jupiter_radius, jupiter_radius));
	jupiter->setSphere(model_jupiter);
	jupiter->showOrbit(true);
	sun->addSubStar(jupiter);


	// saturn
	osg::ref_ptr<Planet> saturn = new Planet(saturn_rotation, saturn_revolution
		, osg::Vec3(saturn_orbitRadius, 0., 0.)
		, osg::Vec3(saturn_radius, saturn_radius, saturn_radius));
	saturn->setSphere(model_saturn);
	saturn->showOrbit(true);
	sun->addSubStar(saturn);

	// uranus
	osg::ref_ptr<Planet> uranus = new Planet(uranus_rotation, uranus_revolution
		, osg::Vec3(uranus_orbitRadius, 0., 0.)
		, osg::Vec3(uranus_radius, uranus_radius, uranus_radius));
	uranus->setSphere(model_uranus);
	uranus->showOrbit(true);
	sun->addSubStar(uranus);

	// neptune
	osg::ref_ptr<Planet> neptune = new Planet(neptune_rotation, neptune_revolution
		, osg::Vec3(neptune_orbitRadius, 0., 0.)
		, osg::Vec3(neptune_radius, neptune_radius, neptune_radius));
	neptune->setSphere(model_neptune);
	neptune->showOrbit(true);
	sun->addSubStar(neptune);

	osg::BoundingSphere bs = sun->getBound();
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(1.,1.,1.,1.));
	colors->push_back(osg::Vec4(.8,.8,.8,1.));

	sun->addChild(createStar(bs.radius(), 20000, colors.get(), star_size_small));
	sun->addChild(createStar(bs.radius(), 5000, colors.get(), star_size_medium));
	sun->addChild(createStar(bs.radius(), 800, colors.get(), star_size_big));

/*
	sun->addChild(new Meteor(osg::Vec3(200.,200.,200)
		, osg::Vec3(100., 100., 100.)
		, 100000
		, 2
		, 500000
		)
	);
*/
	return sun.get();
}

int main()
{

	osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer;
	
	osg::ref_ptr<osg::Node> root = createSceneGraph1();
	viewer->setSceneData( root.get() );
	viewer->getCamera()->setClearColor(osg::Vec4(0., 0., 0., 1.));

	viewer->run();

	//osgDB::writeNodeFile(*root, "C:\\workspace\\OpenSceneGraph-Data\\planet.osg");

};
