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


//osg::ref_ptr<osg::Geode> create





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
	MyObjectCB() 
	{
		_angle = 0.;
	}
	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		osg::MatrixTransform* mt = dynamic_cast<osg::MatrixTransform*>(node);

		osg::Matrix m;

		m *= osg::Matrix::scale(osg::Vec3(0.2, 0.2, 0.2));
		m *= osg::Matrix::translate(osg::Vec3(100., 0., 0.));
	
		_angle += osg::inDegrees(0.5);
		osg::Quat q;
		osg::Vec3 axis = osg::Vec3(1., 1., 1.);
		q.makeRotate(_angle, axis);
		m *= osg::Matrix::rotate(q);

		mt->setMatrix(m);
	
		traverse(node, nv);
	}
protected:
	virtual ~MyObjectCB() {}
private:

	double _angle;

};

class MyObject : public osg::MatrixTransform
{
public:
	MyObject() 
	{
		osg::Node* n = osgDB::readNodeFile("sphere-material-2.3ds");
		osg::StateSet* state = n->getOrCreateStateSet();
		state->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);
		state->setMode(GL_NORMALIZE, osg::StateAttribute::ON);

		addChild(n);

		setUpdateCallback(new MyObjectCB);
	}

protected:
	virtual ~MyObject() {}

private:


};

/*
int main()
{
	osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer;

	//osg::ref_ptr<osg::Node> root = createSceneGraph();

	osg::ref_ptr<osg::Group> root = new osg::Group;
	root->addChild(createSceneGraph());
	root->addChild(new MyObject);

	viewer->setSceneData(root.get());
	viewer->getCamera()->setClearColor(osg::Vec4(0., 0., 0., 1.));

	viewer->run();

}
*/

/* OpenSceneGraph example, osganimate.
*
*  Permission is hereby granted, free of charge, to any person obtaining a copy
*  of this software and associated documentation files (the "Software"), to deal
*  in the Software without restriction, including without limitation the rights
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*  copies of the Software, and to permit persons to whom the Software is
*  furnished to do so, subject to the following conditions:
*
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
*  THE SOFTWARE.
*/

#include <osg/Notify>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/Geometry>
#include <osg/Geode>

#include <osgUtil/Optimizer>

#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>

#include <osgSim/OverlayNode>

#include <osgViewer/Viewer>
#include <iostream>

osg::AnimationPath* createAnimationPath(const osg::Vec3& center, float radius, double looptime)
{
	// set up the animation path
	osg::AnimationPath* animationPath = new osg::AnimationPath;
	animationPath->setLoopMode(osg::AnimationPath::LOOP);

	int numSamples = 40;
	float yaw = 0.0f;
	float yaw_delta = 2.0f*osg::PI / ((float)numSamples - 1.0f);
	float roll = osg::inDegrees(30.0f);

	float pitch = 0.0f;
	float pitch_delta = 3.0f;

	double time = 0.0f;
	double time_delta = looptime / (double)numSamples;
	for (int i = 0; i < numSamples; ++i)
	{
		//osg::Vec3 position(center + osg::Vec3(sinf(yaw)*radius, cosf(yaw)*radius, 0.0f));
		osg::Vec3 position(center + osg::Vec3(sinf(yaw)*radius, cosf(yaw)*radius, pitch));
		//osg::Quat rotation(osg::Quat(roll, osg::Vec3(0.0, 1.0, 0.0))*osg::Quat(-(yaw + osg::inDegrees(90.0f)), osg::Vec3(0.0, 0.0, 1.0)));
		osg::Quat rotation(osg::Quat(-(yaw + osg::inDegrees(90.0f)), osg::Vec3(0.0, 0.0, 1.0)));
		//osg::Quat rotation(osg::Quat(-(osg::inDegrees(90.0f)), osg::Vec3(0.0, 0.0, 1.0)));

		animationPath->insert(time, osg::AnimationPath::ControlPoint(position, rotation));

		yaw += yaw_delta;
		pitch += pitch_delta;
		time += time_delta;

	}
	return animationPath;
}

osg::Node* createBase(const osg::Vec3& center, float radius)
{



	int numTilesX = 10;
	int numTilesY = 10;

	float width = 2 * radius;
	float height = 2 * radius;

	osg::Vec3 v000(center - osg::Vec3(width*0.5f, height*0.5f, 0.0f));
	osg::Vec3 dx(osg::Vec3(width / ((float)numTilesX), 0.0, 0.0f));
	osg::Vec3 dy(osg::Vec3(0.0f, height / ((float)numTilesY), 0.0f));

	// fill in vertices for grid, note numTilesX+1 * numTilesY+1...
	osg::Vec3Array* coords = new osg::Vec3Array;
	int iy;
	for (iy = 0; iy <= numTilesY; ++iy)
	{
		for (int ix = 0; ix <= numTilesX; ++ix)
		{
			coords->push_back(v000 + dx * (float)ix + dy * (float)iy);
		}
	}

	//Just two colours - black and white.
	osg::Vec4Array* colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f)); // white
	colors->push_back(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f)); // black

	osg::ref_ptr<osg::DrawElementsUShort> whitePrimitives = new osg::DrawElementsUShort(GL_QUADS);
	osg::ref_ptr<osg::DrawElementsUShort> blackPrimitives = new osg::DrawElementsUShort(GL_QUADS);

	int numIndicesPerRow = numTilesX + 1;
	for (iy = 0; iy < numTilesY; ++iy)
	{
		for (int ix = 0; ix < numTilesX; ++ix)
		{
			osg::DrawElementsUShort* primitives = ((iy + ix) % 2 == 0) ? whitePrimitives.get() : blackPrimitives.get();
			primitives->push_back(ix + (iy + 1)*numIndicesPerRow);
			primitives->push_back(ix + iy * numIndicesPerRow);
			primitives->push_back((ix + 1) + iy * numIndicesPerRow);
			primitives->push_back((ix + 1) + (iy + 1)*numIndicesPerRow);
		}
	}

	// set up a single normal
	osg::Vec3Array* normals = new osg::Vec3Array;
	normals->push_back(osg::Vec3(0.0f, 0.0f, 1.0f));

	osg::Geometry* geom = new osg::Geometry;
	geom->setVertexArray(coords);

	geom->setColorArray(colors, osg::Array::BIND_PER_PRIMITIVE_SET);

	geom->setNormalArray(normals, osg::Array::BIND_OVERALL);

	geom->addPrimitiveSet(whitePrimitives.get());
	geom->addPrimitiveSet(blackPrimitives.get());

	osg::Geode* geode = new osg::Geode;
	geode->addDrawable(geom);

	return geode;
}

osg::Node* createMovingModel(const osg::Vec3& center, float radius)
{
	float animationLength = 10.0f;

	osg::AnimationPath* animationPath = createAnimationPath(center, radius, animationLength);

	osg::ref_ptr<osg::Group> model = new osg::Group;

	osg::ref_ptr<osg::Node> glider = osgDB::readRefNodeFile("glider.osgt");
	if (glider)
	{
		const osg::BoundingSphere& bs = glider->getBound();

		float size = radius / bs.radius()*0.3f;
		osg::MatrixTransform* positioned = new osg::MatrixTransform;
		positioned->setDataVariance(osg::Object::STATIC);
		positioned->setMatrix(osg::Matrix::translate(-bs.center())*
			osg::Matrix::scale(size, size, size)*
			osg::Matrix::rotate(osg::inDegrees(-90.0f), 0.0f, 0.0f, 1.0f));

		positioned->addChild(glider);

		osg::PositionAttitudeTransform* xform = new osg::PositionAttitudeTransform;
		xform->setUpdateCallback(new osg::AnimationPathCallback(animationPath, 0.0, 1.0));
		xform->addChild(positioned);

		model->addChild(xform);
	}

	osg::ref_ptr<osg::Node> cessna = osgDB::readRefNodeFile("cessna.osgt");
	if (cessna)
	{
		const osg::BoundingSphere& bs = cessna->getBound();

		float size = radius / bs.radius()*0.3f;
		osg::MatrixTransform* positioned = new osg::MatrixTransform;
		positioned->setDataVariance(osg::Object::STATIC);
		positioned->setMatrix(osg::Matrix::translate(-bs.center())*
			osg::Matrix::scale(size, size, size)*
			osg::Matrix::rotate(osg::inDegrees(180.0f), 0.0f, 0.0f, 1.0f));

		positioned->addChild(cessna);

		osg::ref_ptr<osg::MatrixTransform> xform = new osg::MatrixTransform;
		xform->setUpdateCallback(new osg::AnimationPathCallback(animationPath, 0.0f, 2.0));
		xform->addChild(positioned);

		model->addChild(xform);
	}

#ifndef OSG_GLES2_AVAILABLE
	model->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
#endif

	return model.release();
}

osg::ref_ptr<osg::Group> createModel(bool overlay, osgSim::OverlayNode::OverlayTechnique technique)
{
	osg::Vec3 center(0.0f, 0.0f, 0.0f);
	float radius = 100.0f;

	osg::ref_ptr<osg::Group> root = new osg::Group;

	float baseHeight = center.z() - radius * 0.5;
	osg::ref_ptr<osg::Node> baseModel = createBase(osg::Vec3(center.x(), center.y(), baseHeight), radius);
	osg::ref_ptr<osg::Node> movingModel = createMovingModel(center, radius*0.8f);

	if (overlay)
	{
		osgSim::OverlayNode* overlayNode = new osgSim::OverlayNode(technique);
		overlayNode->setContinuousUpdate(true);
		overlayNode->setOverlaySubgraph(movingModel);
		overlayNode->setOverlayBaseHeight(baseHeight - 0.01);
		overlayNode->addChild(baseModel);
		root->addChild(overlayNode);
	}
	else
	{

		root->addChild(baseModel);
	}

	root->addChild(movingModel);

	return root;
}


osg::Node* createTrans()
{
	/** declare a root node*/
	osg::Group* root = new osg::Group;
	/** declare a Position Node*/
	osg::PositionAttitudeTransform* posCow = new osg::PositionAttitudeTransform;
	root->addChild(posCow);
	/** declare a Matrix Node*/
	osg::MatrixTransform* matrixCow = new osg::MatrixTransform;
	root->addChild(matrixCow);

	osg::Node* cow = osgDB::readNodeFile("cow.osg");

	/**
		When use Position Node and the ReferenceFrame is RELATIVE_RF
		the matrix is Compute  Trans(-pivot) * scale * Rotate * Trans(Pos)
		here the pivot and scale is default,so it means that make rotate firstly.
	*/
	posCow->addChild(cow);
	osg::Quat quat;
	quat.makeRotate(osg::PI_2, osg::Vec3(0.0, 0.0, 1.0));
	posCow->setAttitude(quat);
	posCow->setPosition(osg::Vec3(-10, 0.0, 0.0));

	/**
		when use Matrix Node  you can set the matrix what you want.
		here , it  make trans firstly and then make rotate.
	*/
	matrixCow->addChild(cow);
	quat.makeRotate(osg::DegreesToRadians(60.0), osg::Vec3(0.0, 0.0, 1.0));
	matrixCow->setMatrix(osg::Matrixd::translate(osg::Vec3(10.0, 0.0, 0.0))*osg::Matrixd::rotate(quat));

	return root;
}


int main(int argc, char **argv)
{

	bool overlay = false;
	osg::ArgumentParser arguments(&argc, argv);
	while (arguments.read("--overlay")) overlay = true;

	osgSim::OverlayNode::OverlayTechnique technique = osgSim::OverlayNode::OBJECT_DEPENDENT_WITH_ORTHOGRAPHIC_OVERLAY;
	while (arguments.read("--object")) { technique = osgSim::OverlayNode::OBJECT_DEPENDENT_WITH_ORTHOGRAPHIC_OVERLAY; overlay = true; }
	while (arguments.read("--ortho") || arguments.read("--orthographic")) { technique = osgSim::OverlayNode::VIEW_DEPENDENT_WITH_ORTHOGRAPHIC_OVERLAY; overlay = true; }
	while (arguments.read("--persp") || arguments.read("--perspective")) { technique = osgSim::OverlayNode::VIEW_DEPENDENT_WITH_PERSPECTIVE_OVERLAY; overlay = true; }


	// initialize the viewer.
	osgViewer::Viewer viewer;

	// load the nodes from the commandline arguments.
	osg::ref_ptr<osg::Group> model = createModel(overlay, technique);
	if (!model)
	{
		return 1;
	}

	osg::ref_ptr<osg::Group> test_root = new osg::Group;

	test_root->addChild(createSceneGraph());

	// tilt the scene so the default eye position is looking down on the model.
	osg::ref_ptr<osg::MatrixTransform> rootnode = new osg::MatrixTransform;
	//rootnode->setMatrix(osg::Matrix::rotate(osg::inDegrees(30.0f), 1.0f, 0.0f, 0.0f));
	rootnode->addChild(model);

	test_root->addChild(rootnode);

	// run optimization over the scene graph
	osgUtil::Optimizer optimzer;
	optimzer.optimize(rootnode);

	std::string filename;
	if (arguments.read("-o", filename))
	{
		osgDB::writeNodeFile(*rootnode, filename);
		return 1;
	}

	// set the scene to render
	//viewer.setSceneData(rootnode);
	viewer.setSceneData(test_root);

	viewer.setCameraManipulator(new osgGA::TrackballManipulator());

	// viewer.setUpViewOnSingleScreen(1);

#if 0

	// use of custom simulation time.

	viewer.realize();

	double simulationTime = 0.0;

	while (!viewer.done())
	{
		viewer.frame(simulationTime);
		simulationTime += 0.001;
	}

	return 0;
#else

	// normal viewer usage.
	return viewer.run();

#endif
}

