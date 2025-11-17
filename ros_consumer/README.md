## Formato da mensagem ROS personalizada

```
SkeletonList (Mensagem Principal)
  ├── skeletons [LISTA]
  │   └── Skeleton 1
  │       ├── skeleton_id: int32
  │       └── joints [LISTA]
  │           ├── Joint 1
  │           │   ├── id: int32
  │           │   └── position: geometry_msgs/Point (x, y, z)
  │           ├── Joint 2
  │           │   ├── id: int32
  │           │   └── position: geometry_msgs/Point (x, y, z)
  │           └── ... (etc.)
  │   └── Skeleton 2
  │       ├── ...
  │   
  │   └── Skeleton 3
```

*** VERIFICAR GEMINI PARA ENTENDER COMO PERSONALIZAR MENSAGEM ***