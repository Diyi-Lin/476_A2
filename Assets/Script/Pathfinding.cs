using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Specialized;

public class Pathfinding : MonoBehaviour
{
    public bool debug;
    public bool cluster;
    [SerializeField] private GridGraph graph;

    public delegate float Heuristic(Transform start, Transform end);

    public static GridGraphNode startNode;
    public static GridGraphNode goalNode;
    public GameObject openPointPrefab;
    public GameObject closedPointPrefab;
    public GameObject pathPointPrefab;
    public GameObject player;

    //cluster table
    private float[,] cTable = new float[9,9]
    {
        {0,15,17,1,13,1,8,12,9 },
        {15,0,17,16,1,8,1,21,7},
        {17,17,0,8,8,18,19,1,1},
        {1,16,8,0,14,1,9,3,1},
        {13,1,8,14,0,6,1,12,1},
        {1,8,18,1,6,0,1,13,10},
        {8,1,19,9,1,1,0,22,12},
        {12,21,1,3,12,13,22,0,1},
        {9,7,1,1,1,10,12,1,0 },
    };

    private static List<GridGraphNode> OutputPath = new List<GridGraphNode>();
    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.Mouse0))
        {
            if (Physics.Raycast(Camera.main.ScreenPointToRay(Input.mousePosition), out RaycastHit hit, Mathf.Infinity, LayerMask.GetMask("Node")))
            {
                if (startNode != null && goalNode != null)
                {
                    startNode = null;
                    goalNode = null;
                    ClearPoints();
                }

                if (startNode == null)
                {
                    startNode = hit.collider.gameObject.GetComponent<GridGraphNode>();

                }
                else if (goalNode == null)
                {
                    goalNode = hit.collider.gameObject.GetComponent<GridGraphNode>();

                    // TODO: use an admissible heuristic and pass it to the FindPath function
                    OutputPath = FindPath(startNode, goalNode, mHeuristic);
                }
            }
        }
    }

    public static List<GridGraphNode> getOutputList()
    {
        return OutputPath;
    }

    public float mHeuristic(Transform start, Transform goal)
    {
        return Math.Abs(start.position.x - goal.position.x) + Math.Abs(start.position.z - goal.position.z);
    }

    public float cHeuristic(Transform start, Transform goal)
    {
        float x1 = start.position.x;
        float z1 = start.position.z;

        float x2 = goal.position.x;
        float z2 = goal.position.z;

        int c1=10,c2=10;

        if (x1 < -17 && z1 < 0)//A
        {
            c1 = 0;
        }
        if (x1>12 && z1<-17)// B
        {
            c1 = 1;
        }
        if (x1>-3 && z1 > 37)// C
        {
            c1 = 2;
        }
        if (x1<-7 && 2<z1 && z1<23)// D
        {
            c1 = 3;
        }
        if (x1 > -3&& -13 < z1 && z1 < 13)//E
        {
            c1 = 4;
        }
        if (-13 < x1 && x1 < -7 && z1 < 0)//F
        {
            c1 = 5;
        }
        if (-3 < x1 && x1 < 12&& z1 < -17)//G
        {
            c1 = 6;
        }
        if (x1 < -7&& z1 > 27)//H
        {
            c1 = 7;
        }
        if (x1 > -3&& 17 < z1 && z1 < 33)//I
        {
            c1 = 8;
        }
        if (x2 < -17 && z2 < 0)//A
        {
            c2 = 0;
        }
        if (x2 > 12 && z2 < -17)// B
        {
            c2 = 1;
        }
        if (x2 > -3 && z2 > 37)// C
        {
            c2 = 2;
        }
        if (x2 < -7 && 2 < z2 && z2 < 23)// D
        {
            c2 = 3;
        }
        if (x2 > -3 && -13 < z2 && z2 < 13)//E
        {
            c2 = 4;
        }
        if (-13 < x2 && x2 < -7 && z2 < 0)//F
        {
            c2 = 5;
        }
        if (-3 < x2 && x2< 12 && z2 < -17)//G
        {
            c2 = 6;
        }
        if (x2 < -7 && z2 > 27)//H
        {
            c2 = 7;
        }
        if (x2 > -3 && 17 < z2 && z2 < 33)//I
        {
            c2 = 8;
        }
        
        Debug.Log(c1.ToString()+" , "+c2.ToString());

        return cTable[c1, c2]*5 + mHeuristic(start, goal);
    }

    public List<GridGraphNode> FindPath(GridGraphNode start, GridGraphNode goal, Heuristic heuristic = null, bool isAdmissible = true)
    {
        if (graph == null) return new List<GridGraphNode>();

        // if no heuristic is provided then set heuristic = 0
        if (heuristic == null) heuristic = (Transform s, Transform e) => 0;

        List<GridGraphNode> path = null;
        bool solutionFound = false;

        // dictionary to keep track of g(n) values (movement costs)
        Dictionary<GridGraphNode, float> gnDict = new Dictionary<GridGraphNode, float>();
        gnDict.Add(start, default);

        // dictionary to keep track of f(n) values (movement cost + heuristic)
        Dictionary<GridGraphNode, float> fnDict = new Dictionary<GridGraphNode, float>();
        fnDict.Add(start, heuristic(start.transform, goal.transform) + gnDict[start]);

        // dictionary to keep track of our path (came_from)
        Dictionary<GridGraphNode, GridGraphNode> pathDict = new Dictionary<GridGraphNode, GridGraphNode>();
        pathDict.Add(start, null);

        List<GridGraphNode> openList = new List<GridGraphNode>();
        openList.Add(start);

        OrderedDictionary closedODict = new OrderedDictionary();

        while (openList.Count > 0)
        {
            // mimic priority queue and remove from the back of the open list (lowest fn value)
            GridGraphNode current = openList[openList.Count - 1];
            openList.RemoveAt(openList.Count - 1);

            closedODict[current] = true;
            Debug.Log(current);
            Debug.Log(closedODict[current]);

            // early exit
            if (current == goal && isAdmissible)
            {
                solutionFound = true;
                break;
            }
            else if (closedODict.Contains(goal))
            {
                // early exit strategy if heuristic is not admissible (try to avoid this if possible)
                float gGoal = gnDict[goal];
                bool pathIsTheShortest = true;

                foreach (GridGraphNode entry in openList)
                {
                    if (gGoal > gnDict[entry])
                    {
                        pathIsTheShortest = false;
                        break;
                    }
                }

                if (pathIsTheShortest) break;
            }

            List<GridGraphNode> neighbors = graph.GetNeighbors(current);
            foreach (GridGraphNode n in neighbors)
            {
                float movement_cost = 5;
                // TODO

                // if neighbor is in closed list then skip
                // ...
                if (closedODict.Contains(n))
                {
                    continue;
                }

                // find gNeighbor (g_next)
                else
                {
                    if (openList.Contains(n))
                    {
                        continue;
                    }
                    else
                    {
                        if (Math.Abs(n.transform.position.x - current.transform.position.x) + Math.Abs(n.transform.position.y - current.transform.position.y) > 5)
                        {
                            //cost 2 on diagonal edges
                            gnDict.Add(n, gnDict[current] + 2*movement_cost);
                        }
                        //cost 1 on orthogonal edges
                        else gnDict.Add(n, gnDict[current] + movement_cost);
                    }

                }

                // if needed: update tables, calculate fn, and update open_list using FakePQListInsert() function
                if (cluster)
                {
                    fnDict.Add(n, cHeuristic(n.transform, goal.transform) + gnDict[n]);
                }
                else fnDict.Add(n, mHeuristic(n.transform, goal.transform) + gnDict[n]);
                FakePQListInsert(openList, fnDict, n);
                pathDict.Add(n, current);
            }
        }

        // if the closed list contains the goal node then we have found a solution
        if (!solutionFound && closedODict.Contains(goal))
            solutionFound = true;

        if (solutionFound)
        {
            // TODO
            // create the path by traversing the previous nodes in the pathDict
            // starting at the goal and finishing at the start
            path = new List<GridGraphNode>();

            // ...
            path.Add(goal);
            while (!path[path.Count - 1].Equals(start))
            {
                path.Add(pathDict[path[path.Count - 1]]);
            }

            // reverse the path since we started adding nodes from the goal 
            path.Reverse();
        }

        if (debug)
        {
            ClearPoints();

            List<Transform> openListPoints = new List<Transform>();
            foreach (GridGraphNode node in openList)
            {
                openListPoints.Add(node.transform);
            }
            SpawnPoints(openListPoints, openPointPrefab, Color.magenta);

            List<Transform> closedListPoints = new List<Transform>();
            foreach (DictionaryEntry entry in closedODict)
            {
                GridGraphNode node = (GridGraphNode)entry.Key;
                if (solutionFound && !path.Contains(node))
                    closedListPoints.Add(node.transform);
            }
            SpawnPoints(closedListPoints, closedPointPrefab, Color.red);

            if (solutionFound)
            {
                List<Transform> pathPoints = new List<Transform>();
                foreach (GridGraphNode node in path)
                {
                    pathPoints.Add(node.transform);
                }
                SpawnPoints(pathPoints, pathPointPrefab, Color.green);
            }
        }

        return path;
    }

    private void SpawnPoints(List<Transform> points, GameObject prefab, Color color)
    {
        for (int i = 0; i < points.Count; ++i)
        {
#if UNITY_EDITOR
            // Scene view visuals
            points[i].GetComponent<GridGraphNode>()._nodeGizmoColor = color;
#endif

            // Game view visuals
            GameObject obj = Instantiate(prefab, points[i].position, Quaternion.identity, points[i]);
            obj.name = "DEBUG_POINT";
            obj.transform.localPosition += Vector3.up * 0.5f;
        }
    }

    private void ClearPoints()
    {
        foreach (GridGraphNode node in graph.nodes)
        {
            for (int c = 0; c < node.transform.childCount; ++c)
            {
                node._nodeGizmoColor = Color.black;

                if (node.transform.GetChild(c).name == "DEBUG_POINT")
                {
                    Destroy(node.transform.GetChild(c).gameObject);
                }
            }
        }
    }

    /// <summary>
    /// mimics a priority queue here by inserting at the right position using a loop
    /// not a very good solution but ok for this lab example
    /// </summary>
    /// <param name="pqList"></param>
    /// <param name="fnDict"></param>
    /// <param name="node"></param>
    private void FakePQListInsert(List<GridGraphNode> pqList, Dictionary<GridGraphNode, float> fnDict, GridGraphNode node)
    {
        if (pqList.Count == 0)
            pqList.Add(node);
        else
        {
            for (int i = pqList.Count - 1; i >= 0; --i)
            {
                if (fnDict[pqList[i]] > fnDict[node])
                {
                    pqList.Insert(i + 1, node);
                    break;
                }
                else if (i == 0)
                    pqList.Insert(0, node);
            }
        }
    }
}