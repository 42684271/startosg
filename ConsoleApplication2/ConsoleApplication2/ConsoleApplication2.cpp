// ConsoleApplication2.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "pch.h"
#include <iostream>

#include <osg/Node>
#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
#include <osg/NodeVisitor>
#include <osg/StateSet>

//using namespace std;



class SunCB : public osg::NodeCallback
{
public:

	SunCB() : _angle( 0.0 ) {}
	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		osg::MatrixTransform* mt = dynamic_cast<osg::MatrixTransform*>(node);

		osg::Matrix m;
		m.makeRotate(_angle, osg::Vec3(0., 1., 0.) );
		mt->setMatrix(m);

		_angle += 0.00125;

		traverse(node, nv);
	}

protected:
	double _angle;
};

class EarthCB : public osg::NodeCallback
{
public:

	EarthCB() : _angleRotation(0.0), _angleRevolution(0.0) {}
	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		osg::MatrixTransform* mt = dynamic_cast<osg::MatrixTransform*>(node);
		
		osg::MatrixTransform* parent = dynamic_cast<osg::MatrixTransform*>( mt->getParent(0) );


		osg::Matrix mtParent = parent->getMatrix();
		
		// -1
		osg::Quat q = mtParent.getRotate();		
		double aParent;
		osg::Vec3 v;
		q.getRotate(aParent, v);
		osg::Matrix mRotRevParent;
		mRotRevParent.makeRotate(-aParent, v);

		// -2
		//osg::Matrix m = osg::Matrix::inverse(mtParent);
		osg::Matrix m = parent->getInverseMatrix();

		osg::Matrix mT , mS, mR, mRevolution;
		mT.makeTranslate(osg::Vec3(100. ,0. ,0.));
		mS.makeScale(osg::Vec3(.5, .5, .5));
		mR.makeRotate(_angleRotation, osg::Vec3(0., 1., 0.));
		mRevolution.makeRotate(_angleRevolution, osg::Vec3(0., 1., 0.));


		mt->setMatrix(mS * mR * mT * mRevolution * m);

		//mt->setMatrix(m* mS * mR * mT * mRevolution);
		//mt->setMatrix(mS * mR * mT * mRotRevParent);
		//mt->setMatrix(mS * mR * mT);
		//mt->setMatrix(mS * mR * mT * mRotRevParent * mRevolution);
		//mt->setMatrix(mS * mR * mT * m * mRevolution);

		_angleRotation += 0.005;
		_angleRevolution += 0.0005;

		traverse(node, nv);
	}

protected:
	double _angleRotation, _angleRevolution;
};

osg::ref_ptr<osg::Node> createSceneGraph()
{
	osg::ref_ptr<osg::Node> model = osgDB::readNodeFile( "sphere-ems-yellow.3ds" );
	if ( !model.valid() )
	{
		osg::notify(osg::FATAL) << "Open model file error!" << std::endl;
		exit( 1 );
	}

	//model->

	osg::StateSet* state = model->getOrCreateStateSet();
	state->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);

	osg::ref_ptr<osg::MatrixTransform> mtSun = new osg::MatrixTransform;
	mtSun->setUpdateCallback(new SunCB);
	mtSun->addChild( model );

// 	state = mtSun->getOrCreateStateSet();
// 	state->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);

	osg::ref_ptr<osg::MatrixTransform> mtEarth = new osg::MatrixTransform;
	//mtEarth->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	mtEarth->setUpdateCallback(new EarthCB);
	mtEarth->addChild(model);

// 	state = mtSun->getOrCreateStateSet();
// 	state->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);

	mtSun->addChild(mtEarth);

	//return model.get();
	return mtSun.get();
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
		_move = new osg::MatrixTransform;
		_move->setUpdateCallback(new PlanetCB(_angularSpeedRot, 0, osg::Vec3(0., 0., 0.), _equatorRadius));
		addChild(_move);
	}

	virtual void addSubStar(osg::Node* star) {	addChild(star);	}
	virtual void setSphere(osg::Node* sphere) { _move->addChild(sphere);  }
	virtual void makeMovement(osg::NodeCallback* cb) { setUpdateCallback(cb);  }

	//~Planet();

private:
	osg::ref_ptr<osg::MatrixTransform> _move;
	double _angularSpeedRot;
	double _angularSpeedRevo;

	osg::Vec3	_orbitRadius;
	osg::Vec3	_equatorRadius;
};


osg::ref_ptr<osg::Node> createSceneGraph1()
{

	osg::ref_ptr<osg::Node> model = osgDB::readNodeFile("sphere-ems-yellow.3ds");
	if (!model.valid())
	{
		osg::notify(osg::FATAL) << "Open model file error!" << std::endl;
		exit(1);
	}

	//factor - to adapt to frame update.
	double dOrbitRadiusFactor	= 20.0;
	double dEquatorRadiusFactor	= 1.0;
	double dRotFactor		= 0.00005;
	double dRevo	= 0.000005;

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
	double moon_orbitRadius = 10.0	*dOrbitRadiusFactor;

	//double sun_radius = 285.24 *dRadiusFactor;
	double sun_radius = 28.524 *dEquatorRadiusFactor;
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

	double sun_revolution = 0.0 *dRevo;
	double mercury_revolution	= 685.85 *dRevo;
	double venus_revolution		= 268.48 *dRevo;
	double earth_revolution		= 165.28 *dRevo;
	double mars_revolution		= 87.81  *dRevo;
	double jupiter_revolution	= 13.93  *dRevo;
	double saturn_revolution	= 5.56   *dRevo;
	double uranus_revolution	= 1.96   *dRevo;
	double neptune_revolution	= 1.00   *dRevo;
	double moon_revolution		= 92.00  *dRevo;


	// planet model
	osg::StateSet* state = model->getOrCreateStateSet();
	state->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);

	// sun
	//osg::ref_ptr<Planet> sun = new Planet(0., 0., osg::Vec3(0.,0.,0.), osg::Vec3(1., 1., 1.));
	osg::ref_ptr<Planet> sun = new Planet(sun_rotation, sun_revolution
		, osg::Vec3(sun_orbitRadius, 0., 0.)
		, osg::Vec3(sun_radius, sun_radius, sun_radius));
		//, osg::Vec3(mercury_orbitRadius, 0., 0.), osg::Vec3(mercury_radius, mercury_radius, mercury_radius));
	sun->setSphere(model);


	// mercury
	osg::ref_ptr<Planet> mercury = new Planet(mercury_rotation, mercury_revolution
		, osg::Vec3(mercury_orbitRadius, 0., 0.)
		, osg::Vec3(mercury_radius, mercury_radius, mercury_radius));
	mercury->setSphere(model);
	sun->addSubStar(mercury);


	// venus
	osg::ref_ptr<Planet> venus = new Planet(venus_rotation, venus_revolution
		, osg::Vec3(venus_orbitRadius, 0., 0.)
		, osg::Vec3(venus_radius, venus_radius, venus_radius));
	venus->setSphere(model);
	sun->addSubStar(venus);


	osg::ref_ptr<Planet> earth = new Planet(earth_rotation, earth_revolution
		, osg::Vec3(earth_orbitRadius, 0., 0.)
		, osg::Vec3(earth_radius, earth_radius, earth_radius));
	earth->setSphere(model);
	sun->addSubStar(earth);

	{
		// moon
// 		osg::ref_ptr<Planet> moon = new Planet(earth_rotation, earth_revolution
// 			, osg::Vec3(moon_orbitRadius, 0., 0.)
// 			, osg::Vec3(moon_radius, moon_radius, moon_radius));
// 		moon->setSphere(model);
//  		earth->addSubStar(moon);
		osg::ref_ptr<Planet> moon = new Planet(moon_rotation, moon_revolution
			, osg::Vec3(moon_orbitRadius, 0., 0.)
			, osg::Vec3(moon_radius, moon_radius, moon_radius));
		moon->setSphere(model);
		earth->addSubStar(moon);

	}

	// mars
	osg::ref_ptr<Planet> mars = new Planet(mars_rotation, mars_revolution
		, osg::Vec3(mars_orbitRadius, 0., 0.)
		, osg::Vec3(mars_radius, mars_radius, mars_radius));
	mars->setSphere(model);
	sun->addSubStar(mars);


	// jupiter
	osg::ref_ptr<Planet> jupiter = new Planet(jupiter_rotation, jupiter_revolution
		, osg::Vec3(jupiter_orbitRadius, 0., 0.)
		, osg::Vec3(jupiter_radius, jupiter_radius, jupiter_radius));
	jupiter->setSphere(model);
	sun->addSubStar(jupiter);


	// saturn
	osg::ref_ptr<Planet> saturn = new Planet(saturn_rotation, saturn_revolution
		, osg::Vec3(saturn_orbitRadius, 0., 0.)
		, osg::Vec3(saturn_radius, saturn_radius, saturn_radius));
	saturn->setSphere(model);
	sun->addSubStar(saturn);

	// uranus
	osg::ref_ptr<Planet> uranus = new Planet(uranus_rotation, uranus_revolution
		, osg::Vec3(uranus_orbitRadius, 0., 0.)
		, osg::Vec3(uranus_radius, uranus_radius, uranus_radius));
	uranus->setSphere(model);
	sun->addSubStar(uranus);

	// neptune
	osg::ref_ptr<Planet> neptune = new Planet(neptune_rotation, neptune_revolution
		, osg::Vec3(neptune_orbitRadius, 0., 0.)
		, osg::Vec3(neptune_radius, neptune_radius, neptune_radius));
	neptune->setSphere(model);
	sun->addSubStar(neptune);

	return sun.get();
}

int main()
{

	osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer;

	viewer->setSceneData(createSceneGraph1().get());

	viewer->run();

};
