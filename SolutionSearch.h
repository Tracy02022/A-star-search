
/*implemented by Dongyu Li*/
#pragma once
#include<vector> 
#include<algorithm> 
#include<queue>
#include<functional>
#include<stack>
#include<fstream>
#include<map>
#include<iostream>
#include<ostream>
using namespace std;
class SolutionSearch
{
private:
	/*The node structure is used to store each state information*/
	struct Node
	{
		/*value stores data in 8 positions*/
		int value[9];
		/*This is the number eight in current state*/
		int Operators;
		int f;
		int gf;
		/*used for map*/
		int id;
		/*overload '<' used for map*/
		bool operator <(const Node& other)const
		{
			return(id < other.id);
		}
		/*overload '==' used for comparison*/
		bool operator==(const Node& a) const
		{
			return (value[0] == a.value[0] && value[1] == a.value[1] && value[2] == a.value[2] &&
				value[3] == a.value[3] && value[4] == a.value[4] &&
				value[5] == a.value[5] && value[6] == a.value[6] &&
				value[7] == a.value[7] && value[8] == a.value[8]);
		}
		bool operator >(const Node& b)const
		{
			return f < b.f;
		}		
	};	
	int ID;
	/*store goal state*/
	Node goal;
	/*store current state*/
	Node current;
	/*store all the parents in A* search*/
	map<Node , Node> M1;
	map<Node, Node>::iterator iter;
	/*open list*/
	vector<Node> q;
	vector<Node>::iterator result1;
	/*close list*/
	vector<Node> repeat_list;
	vector<Node>::iterator result2;
public:
	SolutionSearch(void);
	~SolutionSearch(void);
	/*heuristic function based on manhatton distance*/
	int HeuristicFunction(int pos1,int pos2);
	/*swap two positions in A* search */
	void swap(int pos1, int pos2);
	/*make comparison in repeatlist*/
	void Compare(int Oper);
	bool AStarSearch(int *data, vector<int> &solution);	
};

