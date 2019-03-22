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


int main()
{
    
	osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer;

	viewer->setSceneData( createSceneGraph().get() );

	viewer->run();

	//std::cout << "Hello World!\n"; 
}



