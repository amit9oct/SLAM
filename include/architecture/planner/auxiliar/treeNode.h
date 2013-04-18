
#ifndef ___TREE_NODE__
#define ___TREE_NODE__

#include "robotTypes.h"

#define GATEWAY_NODE 			0
#define LEAF_TYPE_FRONTIER		1
#define LEAF_TYPE_PRECISE_POSE		2

/**
* @brief Nodes that form a tree that partitions the environment for the high level planner in the hybrid arquitecture
*
*/
class treeNode{

private:
	treeNode* parent;
	int nchildren;
	treeNode** children;
	
	double ipoints;
	int nbots;
	double value;
	int nextNodeInPlan;

	int x;
	int y;
	double cost;
	double localCost;

	int node_type;

	int numrobots;
	bool* robots;

public:
	treeNode(int type, int nrobots);
	virtual ~treeNode();

	void setCell(point pos);
	void setiPoints(double n);
	void setnBots(int n);
	double getiPoints() const;
	int getnBots() const;

	int getNChildren() const;
	
	double getValue() const;
	void setValue(double value);

	void setParent(treeNode& parent);
	void addChildren(treeNode& child);

	treeNode& getChildren(int number) const ;
	treeNode& getParent() const;

	void setCost(double val);
	double getCost() const;
	void setLocalCost(double val);
	double getLocalCost() const;

	int getX() const;
	int getY() const;

	bool isLeaf() const;
	int getNodeType() const;
	bool isRoot() const;

	void clearRobots();
	bool isRobot(int robot) const;
	void setRobot(int robot);

	static bool orderByValue(const treeNode* a, const treeNode* b);
	static bool orderByCost(const treeNode* a, const treeNode* b);

	int getNextNodeInPlan() const;
	void setNextNodeInPlan(int n);
};

inline void treeNode::setCell(point pos)				{x = pos.x; y = pos.y;}
inline void treeNode::setiPoints(double n)				{ipoints = n;}
inline double treeNode::getiPoints() const 				{return ipoints;}
inline void treeNode::setnBots(int n)					{nbots = n;}
inline int treeNode::getnBots() const 					{return nbots;}
inline void treeNode::setValue(double val)				{value = val;}
inline double treeNode::getValue() const				{return value;}
inline int treeNode::getNChildren() const 				{return nchildren;}
inline void treeNode::setParent(treeNode& p)				{parent = &p;}
inline bool treeNode::isRoot() const					{return(parent==0);}
inline int treeNode::getX() const 					{return x;}
inline int treeNode::getY() const 					{return y;}
inline bool treeNode::isLeaf() const 					{return (node_type != GATEWAY_NODE);}
inline int treeNode::getNodeType() const				{return node_type;}
inline void treeNode::setCost(double val)				{cost = val;}
inline double treeNode::getCost() const 				{return cost;}
inline void treeNode::setLocalCost(double val)				{localCost = val;}
inline double treeNode::getLocalCost() const 				{return localCost;}
inline int treeNode::getNextNodeInPlan() const				{return nextNodeInPlan;}
inline void treeNode::setNextNodeInPlan(int n)				{nextNodeInPlan = n;}

#endif

