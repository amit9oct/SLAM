	
	
#include "treeNode.h"
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

treeNode::treeNode(int type, int nrobots):
	parent(0),
	nchildren(0),
	children(0),

	ipoints(0),
	nbots(0),
	value(0.0f),
	nextNodeInPlan(-1),

	node_type(type),

	numrobots(nrobots),
	robots(new bool[nrobots])
{

}

treeNode::~treeNode(){
	for (int n=0; n<nchildren; n++){
		delete children[n];
	}
	if (children)
		free(children);
	delete[] robots;
}

// TODO: CHANGE THIS REALLOC TO A STD::VECTOR WITH INITIAL RESERVATION
void treeNode::addChildren(treeNode& child){

	child.setParent(*this);
	nchildren++;

	children = (treeNode**) realloc(children, nchildren*sizeof(treeNode*));

	children[nchildren-1] = &child;
}

treeNode& treeNode::getChildren(int number) const {
	assert(number < nchildren);
	return *(children[number]);
}

treeNode& treeNode::getParent() const{
	assert(parent);
	return (*parent);
}

void treeNode::clearRobots(){
	for (int i = 0; i< numrobots; i++)
		robots[i] = false;
}

bool treeNode::isRobot(int robot) const {
	if (robot < numrobots)
		return robots[robot];
	else
		return false;
}

void treeNode::setRobot(int robot){
	if (robot < numrobots)
		robots[robot] = true;	
}

bool treeNode::orderByValue(const treeNode* a, const treeNode* b){
	//printf("ordenando por valor %f y %f\n", ((treeNode*)a)->getValue(), ((treeNode*)b)->getValue());
	if (a == b) return false;
	return (((treeNode*)a)->getValue() > ((treeNode*)b)->getValue());
}

bool treeNode::orderByCost(const treeNode* a, const treeNode* b){
	//printf("ordenando por coste %f y %f\n", ((treeNode*)a)->getCost(), ((treeNode*)b)->getCost());
	if (a == b) return false;
	else return (((treeNode*)a)->getCost() > ((treeNode*)b)->getCost());
}


