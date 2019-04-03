// ConsoleApplication4.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "pch.h"
#include <iostream>


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

osg::ref_ptr<osg::Node> createSceneGraph()
{

#define AXIS_LENGTH (500)
#define LABEL_SIZE	 (2)


	osg::ref_ptr<osg::Geode> geod = new osg::Geode;
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;

	osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array;
	osg::ref_ptr<osg::Vec3Array> n = new osg::Vec3Array;

	// x axis
	v->push_back(osg::Vec3(.0,.0,.0));
	v->push_back(osg::Vec3(AXIS_LENGTH, 0., 0.));

	// y axis
	v->push_back(osg::Vec3(.0, .0, .0));
	v->push_back(osg::Vec3(.0, AXIS_LENGTH, 0.));

	// z axis
	v->push_back(osg::Vec3(.0, .0, .0));
	v->push_back(osg::Vec3(0., 0., AXIS_LENGTH));

	c->push_back(osg::Vec4(1., .0, .0, 1.));
	c->push_back(osg::Vec4(.0, 1., .0, 1.));
	c->push_back(osg::Vec4(.0, .0, 1., 1.));

	geom->setVertexArray(v.get());
	geom->setColorArray(c.get());
	geom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2));
 	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 2, 2));
 	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 4, 2));

	osg::StateSet* stateset = new osg::StateSet;
	osg::LineWidth* linewidth = new osg::LineWidth();
	linewidth->setWidth(2.);
	stateset->setAttributeAndModes(linewidth, osg::StateAttribute::ON);
	stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	geom->setStateSet(stateset);

	geod->addDrawable(geom);

	
	// label
	osg::ref_ptr<osg::Geometry> geom_label = new osg::Geometry;

	osg::ref_ptr<osg::Vec3Array> v_label = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec4Array> c_label = new osg::Vec4Array;
	osg::ref_ptr<osg::Vec3Array> n_label = new osg::Vec3Array;

	for (int iLabel = 0; iLabel < AXIS_LENGTH; iLabel+=100)
	{
		v_label->push_back(osg::Vec3(iLabel, .0, .0));
		v_label->push_back(osg::Vec3(.0, iLabel, .0));
		v_label->push_back(osg::Vec3(.0, .0, iLabel));
	}

	c_label->push_back(osg::Vec4(1., 1., 1., 1.));
	n_label->push_back(osg::Vec3(1.,1.,1.));

	geom_label->setVertexArray(v_label.get());
	geom_label->setColorArray(c_label.get());
	geom_label->setColorBinding(osg::Geometry::BIND_OVERALL);
	geom_label->setNormalArray(n_label.get());
	geom_label->setNormalBinding(osg::Geometry::BIND_OVERALL);

	osg::StateSet* st_label = geom_label->getOrCreateStateSet();
	osg::Point* point = new osg::Point;
	point->setSize(3.);
	st_label->setAttributeAndModes(point, osg::StateAttribute::ON);

	geom_label->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, (AXIS_LENGTH) / 100 * 3));

	geod->addDrawable(geom_label);


	return geod.get();

}

class MyObjectCB : public osg::NodeCallback
{
public:
	MyObjectCB() {}
	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		traverse(node, nv);
	}
protected:
	virtual ~MyObjectCB() {}
private:
};

class MyObject : public osg::MatrixTransform
{
public:
	MyObject() 
	{
		
	}

protected:
	virtual ~MyObject() {}

private:


};

int main()
{
	osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer;

	osg::ref_ptr<osg::Node> root = createSceneGraph();
	viewer->setSceneData(root.get());
	viewer->getCamera()->setClearColor(osg::Vec4(0., 0., 0., 1.));

	viewer->run();

}
