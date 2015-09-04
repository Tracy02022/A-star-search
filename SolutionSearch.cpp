#include "SolutionSearch.h"
SolutionSearch::SolutionSearch(void)
{
	/*initialization*/
	q.empty();	

	/*every node has its own id, no two nodes have the same id*/
	ID = 0;
	repeat_list.empty();

	/*initialization*/
	for (int i = 0; i < 9; i++)
	{
		goal.value[i] = i;
	}		
}
SolutionSearch::~SolutionSearch(void)
{	
	M1.clear();
	q.clear();
	repeat_list.clear();
}
/*heuristic function based on Manhattan distance*/
int SolutionSearch::HeuristicFunction(int pos1, int pos2)
{
	int H[9][9];
	H[0][0] = H[1][1] = H[2][2] = H[3][3] = H[4][4] = H[5][5] = H[6][6] = H[7][7] = H[8][8] = 0;

	H[0][1] = H[0][3] = H[1][0] = H[1][2] = H[1][4] = H[2][1] = H[2][5] = H[3][0] = H[3][4] = H[3][6] = 1;
	H[4][1] = H[4][3] = H[4][5] = H[4][7] = H[5][2] = H[5][4] = H[5][8] = H[6][3] = H[6][7] = 1;
	H[7][6] = H[7][4] = H[7][8] = H[8][5] = H[8][7] = 1;

	H[0][2] = H[0][4] = H[0][6] = H[1][3] = H[1][5] = H[1][7] = H[2][0] = H[2][4] = H[2][8] = 2;
	H[3][1] = H[3][7] = H[3][5] = H[4][0] = H[4][2] = H[4][6] = H[4][8] = H[5][1] = H[5][3] = H[5][7] = 2;
	H[6][0] = H[6][4] = H[6][8] = H[7][1] = H[7][3] = H[7][5] = H[8][2] = H[8][4] = H[8][6] = 2;

	H[0][5] = H[0][7] = H[1][6] = H[1][8] = H[2][3] = H[2][7] = H[3][2] = H[3][8] = H[5][0] = H[5][6] = 3;
	H[6][1] = H[6][5] = H[7][0] = H[7][2] = H[8][1] = H[8][3] = 3;

	H[0][8] = H[2][6] = H[6][2] = H[8][0] = 4;

	return H[pos1][pos2];

}
/*swap two values in two positions in A* search */
void SolutionSearch::swap(int pos1, int pos2)
{
	int temp;
	current = q.front();
	current.gf = q.front().gf+1;
	temp = current.value[pos1];
	current.value[pos1] =current.value[pos2];
	current.value[pos2] = temp; 
	int h = 0;
	/*for each node compute the h() value*/
	for (int i = 0; i < 9; i++)
	{
		h += HeuristicFunction(i, current.value[i]);
	}
	current.f = current.gf + h;
}
/*comparison*/
void SolutionSearch::Compare(int Oper)
{
	/*q is the open list should be checked*/
	result1 = find(q.begin(), q.end(), current);
	/*repeat_list is the close list should be checked*/
	result2 = find(repeat_list.begin(), repeat_list.end(), current);
	/*if the node is totally new, add it in the open list*/
	if ((result1 == q.end()) && (result2 == repeat_list.end()))
	{
		Node cur;
		cur = current;
		cur.Operators = Oper;
		ID++;
		cur.id = ID;
		cur.gf = current.gf;
		/*for each node, compute its f() value*/
		cur.f = current.f;
		M1.insert(pair<Node, Node>(cur, q.front()));
		q.push_back(cur);
	}
	/*if the node is in the open list, check the f() value, and update*/
	else if (result1 != q.end())
	{
		/*if f() value if smaller, update the node information*/
		if (current.f < result1->f)
		{
			result1->f = current.f;
			iter = M1.find(current);
			if (iter != M1.end())
			{
				iter->second = q.front();
			}
			else
			{
				ID++;
				current.id = ID;
				M1.insert(pair<Node, Node>(current, q.front()));
			}
		}
	}
	/*if the node is in the close list, which means it has been tracked before*/
	else if(result2 != repeat_list.end())
	{
		/*add it into the open list if and only if the f() value is smaller*/
		if (current.f < result2->f)
		{
			result2->f = current.f;
			q.push_back(current);
			/*remove the updated node from close list*/
			repeat_list.erase(result2);
			iter = M1.find(current);
			if (iter != M1.end())
			{
				iter->second = q.front();
			}
			else
			{
				ID++;
				current.id = ID;
				M1.insert(pair<Node, Node>(current, q.front()));
			}
		}
	}
	
}
/*begin search*/
bool SolutionSearch::AStarSearch(int *data, vector<int> &solution)
{
	/*store reverse path from child to parent*/
	stack<int> revers;
	int t;
	/*store heuristic function value*/
	int h = 0;
	bool g;
	Node node;
	/*store g() value*/
	node.gf = 0;
	/*initialize the first node*/
	for (int i = 0; i < 9; i++)
	{
		node.value[i] = data[i];
		h += HeuristicFunction(i, data[i]);
		if (data[i] == 8)
		{
			t = i;
			node.Operators = t;
		}
	}
	/*compute the f() value for first node*/
	node.f = 0 + h;
	node.id = ID;
	/*put it in the open list*/
	q.push_back(node);
	M1.insert(pair<Node, Node>(node, node));
	while (!q.empty())
	{	
		if (q.front() == goal)
			g = true;
		else
			g = false;
		/*if goal,return the path*/
		/*find the goal until it has been expanded from the open list*/
		if (g)
		{
			revers.push(q.front().Operators);
			iter = M1.find(q.front());
			while (iter != M1.end())
			{
				if (iter->second == node)
					break;
				revers.push(iter->second.Operators);
				iter = M1.find(iter->second);
			}
			while (!revers.empty())
			{
				solution.push_back(revers.top());
				revers.pop();
			}
			return true;
		}
		else
		{
			/*for the node with smallest f() value*/
			t = q.front().Operators;
			/*create its children and check the f() value*/
			/*check the child node to make sure it has not been tracked*/
			switch (t)
			{
			case 0:
				swap(1, 0);
				Compare(1);
				swap(3, 0);
				Compare(3);
				break;
			case 1:
				swap(1, 2);
				Compare(2);
				swap(1, 4);			
				Compare(4);
				swap(1, 0);
				Compare(0);
				break;
			case 2:
				swap(2, 1);			
				Compare(1);
				swap(2, 5);			
				Compare(5);
				break;
			case 3:
				swap(0, 3);			
				Compare(0);
				swap(4, 3);			
				Compare(4);
				swap(6, 3);			
				Compare(6);
				break;
			case 4:
				swap(1, 4);			
				Compare(1);
				swap(3, 4);			
				Compare(3);
				swap(5, 4);			
				Compare(5);
				swap(7, 4);			
				Compare(7);
				break;
			case 5:
				swap(5, 2);
				Compare(2);
				swap(5, 4);			
				Compare(4);
				swap(5, 8);			
				Compare(8);		
				break;
			case 6:
				swap(6, 7);		
				Compare(7);
				swap(6, 3);		
				Compare(3);
				break;
			case 7:
				swap(7, 8);				
				Compare(8);			
				swap(7, 6);				
				Compare(6);
				swap(7, 4);			
				Compare(4);
				break;
			case 8:
				swap(7, 8);			
				Compare(7);
				swap(5, 8);		
				Compare(5);
				break;
			default:
				break;
			}
			repeat_list.push_back(q.front());
			result1 = q.begin();
			q.erase(result1);
			/*sort the open list to find the node with smallest f() value*/
			sort(q.begin(), q.end(), greater<Node>());
		}
	}
	return 1;
}