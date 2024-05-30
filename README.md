# Continuous Trajectory Similarity Search for Online Outlier Detection

The code implements the CTSS algorithm for online outlier detection.



## Supported query modes

1. Single point query in Euclidean space
2. Continuous query in Euclidean space
3. Single point query in Road network space
4. Continuous query in Road network space



## File format

#### Vertex file

when reading the vertex in the file, the structure of each line is required to be:

```.txt
VertexID	Latitude	Longitude
```

Example:

```.txt
0	39.9476009	116.4020469
1	39.9476591	116.4053914
2	39.9319029	116.4025943
...
```

#### Edge file

when reading the edge in the file, the structure of each line is required to be:

```.txt
EdgeID	FirstVertexID	SecondVertexID
```

Example:

```.txt
0	0	1
1	2	701
2	701	3
...
```

#### Reference path file

when reading the reference path in the file, the structure of each line is required to be:

```.txt
EdgeID	FirstVertexID	SecondVertexID
```

Example:

```.txt
1578	1902	1201
2764	1201	114
2765	114	302
...
```

#### Complete path file

when reading the complete path in the file, the structure of each line is required to be:

```.txt
EdgeID	FirstVertexID	SecondVertexID
```

Example:

```.txt
1578	1902	1201
2764	1201	114
2765	114	302
...
```



## Build

```
make
```



## Run

```
./CTSS "input_folder_path" "output_folder_path" "start_position" "stop_position" "delta" "query_mode" "section_length"
```

Example:

```
./CTSS data/beijing/set_14000 data/beijing/out 1 20 0.0081 1 100
```



## Contributors

- Dongxiang Zhang: zhangdongxiang@zju.edu.cn
- Zhihao Chang: changzhihao@zju.edu.cn



## Reference
If you want to use this code, please cite:
```
@article{DBLP:journals/tkde/ZhangCWYTC22,
  author       = {Dongxiang Zhang and
                  Zhihao Chang and
                  Sai Wu and
                  Ye Yuan and
                  Kian{-}Lee Tan and
                  Gang Chen},
  title        = {Continuous Trajectory Similarity Search for Online Outlier Detection},
  journal      = {{IEEE} Trans. Knowl. Data Eng.},
  volume       = {34},
  number       = {10},
  pages        = {4690--4704},
  year         = {2022},
}
```
