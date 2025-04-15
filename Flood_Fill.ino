void wall(uint8_t x, uint8_t y) {
  if(wallFront()){
    coordinates[x][y][o] = 1;
  }
  if(wallLeft()){
    coordinates[x][y][(o+3)%4] = 1;
  }
  if(wallRight()){
    coordinates[x][y][(o+1)%4] = 1;
  }
  
  
  //Do mỗi toạ độ đều lưu vị trí 4 hường tường nên sẽ bình chồng lên nhau, ví dụ ô 0,0 có tường bên phải thì ô 1,0 có tường ở bên trái
  if(coordinates[x][y][0] == 1 && x > 0 && coordinates[x-1][y][2] == 0) {
    coordinates[x-1][y][2] = 1;
  }
  if(coordinates[x][y][1] == 1 && y < maze_size-1 && coordinates[x][y+1][3] == 0) {
    coordinates[x][y+1][3] = 1;
  }
  if(coordinates[x][y][2] == 1 && x < maze_size-1 && coordinates[x+1][y][0] == 0) {
    coordinates[x+1][y][0] = 1;
  }
  if(coordinates[x][y][3] == 1 && y > 0 && coordinates[x][y-1][1] == 0) {
    coordinates[x][y-1][1] = 1;
  }

}


void Oxy() {
  switch (o) {
    case 0:
      x = x - 1;
      break;
    case 1:
      y = y + 1;
      break;
    case 2:
      x = x + 1;
      break;
    case 3:
      y = y - 1;
      break;
    default:
      break;
  }
}

/*
void show_wall(uint8_t x, uint8_t y) {
  if(coordinates[x][y][0] == 1) {
    setWall(x,y,'w');
  }
  if(coordinates[x][y][1] == 1) {
    setWall(x,y,'n');
  }
  if(coordinates[x][y][2] == 1) {
    setWall(x,y,'e');
  }
  if(coordinates[x][y][3] == 1) {
    setWall(x,y,'s');
  }
}
*/

void goal_detemine() {
  
  if((x == maze_size/2-1 || x == maze_size/2) && (y == maze_size/2-1 || y == maze_size/2)) {
    reach_goal = 1;

    for(uint8_t m = maze_size/2-1; m <= maze_size/2; m++) {
      for(uint8_t n = maze_size/2-1; n <= maze_size/2; n++) {
        for(uint8_t z = 0; z <= 3; z++) {
          coordinates[m][n][z] = 1;
        }
      }
    }
    coordinates[x][y][(o+2)%4] = 0;
  }
    
    /*
    coordinates[7][7][0] = 1;
    coordinates[7][8][0] = 1;
    coordinates[7][8][1] = 1;
    coordinates[8][8][1] = 1;
    coordinates[8][8][2] = 1;
    coordinates[8][7][2] = 1;
    coordinates[8][7][3] = 1;
    coordinates[7][7][3] = 1;
    */
  
  

  if(x == maze_size/2-1 && y == maze_size/2-1) {
    if(o == 1) {
      coordinates[maze_size/2-1][maze_size/2-1][3] = 0;
    }else if(o == 2) {
      coordinates[maze_size/2-1][maze_size/2-1][0] = 0;
    }
  }
  if(x == maze_size/2-1 && y == maze_size/2) {
    if(o == 3) {
      coordinates[maze_size/2-1][maze_size/2][1] = 0;
    }else if(o == 2) {
      coordinates[maze_size/2-1][maze_size/2][0] = 0;
    }
  }
  if(x == maze_size/2 && y == maze_size/2) {
    if(o == 3) {
      coordinates[maze_size/2][maze_size/2][1] = 0;
    }else if(o == 0) {
      coordinates[maze_size/2][maze_size/2][2] = 0;
    }
  }
  if(x == maze_size/2 && y == maze_size/2-1) {
    if(o == 0) {
      coordinates[maze_size/2-1][maze_size/2][2] = 0;
    }else if(o == 1) {
      coordinates[maze_size/2-1][maze_size/2][3] = 0;
    }
  }
  
  
  if(reach_goal == 1) {
    for(uint8_t m = maze_size/2-2; m < maze_size/2+2; m++) {
      //log("check lại wall của cột " + String(m));
      for(uint8_t n = maze_size/2-2; n < maze_size/2+2; n++) {
        if(coordinates[x][y][0] == 1 && x > 0 && coordinates[x-1][y][2] == 0) {
          coordinates[x-1][y][2] = 1;
        }
        if(coordinates[x][y][1] == 1 && y < maze_size-1 && coordinates[x][y+1][3] == 0) {
          coordinates[x][y+1][3] = 1;
        }
        if(coordinates[x][y][2] == 1 && x < maze_size-1 && coordinates[x+1][y][0] == 0) {
          coordinates[x+1][y][0] = 1;
        }
        if(coordinates[x][y][3] == 1 && y > 0 && coordinates[x][y-1][1] == 0) {
          coordinates[x][y-1][1] = 1;
        }
        //show_wall(m,n);

      }
    }
  }
    
}


void Flood_Fill() {
  ArduinoQueue<point> oxy(64); //tính nhẩm thì cần lưu được tối thiểu 8*4 = 32 nên để 64 cho chắc
  //Đưa hết các ô về blank (giá trị = 0)
  for(uint8_t m = 0; m < maze_size; m++) {
    for(uint8_t n = 0; n < maze_size; n++) {
      floodfill[m][n] = 0;
      //setText(m,n, "");
    }
  }
  
  //Set các ô đích = 1 và cho vào hàng chờ
  for(uint8_t m = maze_size/2-1; m <= maze_size/2; m++) {
    for(uint8_t n = maze_size/2-1; n <=maze_size/2 ; n++) {
      floodfill[m][n] = 1;
      oxy.enqueue({m,n});
      //setText(m,n, String(floodfill[m][n]));
    }
  }
  
  
  //oxy.enqueue({7,7});
  //oxy.enqueue({7,8});
  //oxy.enqueue({8,8});
  //oxy.enqueue({8,7});
  
  while(!oxy.isEmpty()) {
    a = oxy.getHead().x;
    b = oxy.getHead().y;
    oxy.dequeue();
    
    if(coordinates[a][b][0] == 0 && floodfill[a-1][b] == 0 && a > 0) {
      floodfill[a-1][b] = floodfill[a][b] + 1;
      oxy.enqueue({a-1,b});
    }
    if(coordinates[a][b][1] == 0 && floodfill[a][b+1] == 0 && b < maze_size-1) {
      floodfill[a][b+1] = floodfill[a][b] + 1;
      oxy.enqueue({a,b+1});
    }
    if(coordinates[a][b][2] == 0 && floodfill[a+1][b] == 0 && a < maze_size-1) {
      floodfill[a+1][b] = floodfill[a][b] + 1;
      oxy.enqueue({a+1,b});
    }
    if(coordinates[a][b][3] == 0 && floodfill[a][b-1] == 0 && b > 0) {
      floodfill[a][b-1] = floodfill[a][b] + 1;
      oxy.enqueue({a,b-1});
    }
  }
  
}


//hàm để quay xe theo hướng chỉ định
void change_direction (uint8_t d) {
    switch (d - o) {
        case 1:
            turnRight();
            break;
        case 2:
            // turnRight();
            // turnRight();
            turnAround();
            break;
        case 3:
            turnLeft();
            break;
        case -1:
            turnLeft();
            break;
        case -2:
            // turnLeft();
            // turnLeft();
            turnAround();
            break;
        case -3:
            turnRight();
            break;
    }

    o = d;
}

void Move() {
  
  //Ưu tiên rẽ trái -> đi thắng -> rẽ phải -> lùi
  /*
  if(coordinates[x][y][0] == 0 && x > 0 && floodfill[x-1][y] < floodfill[x][y]) {
    change_direction(0);
  }else if(coordinates[x][y][1] == 0 && y < maze_size-1 && floodfill[x][y+1] < floodfill[x][y]) {
    change_direction(1);
  }else if(coordinates[x][y][2] == 0 && x < maze_size-1 && floodfill[x+1][y] < floodfill[x][y]) {
    change_direction(2);
  }else if(coordinates[x][y][3] == 0 && y > 0 && floodfill[x][y-1] < floodfill[x][y]) {
    change_direction(3);
  }
  */

  //Ưu tiên đi thẳng đi thắng -> rẽ phải -> lùi -> rẽ trái
  /*
  if(coordinates[x][y][1] == 0 && y < maze_size-1 && floodfill[x][y+1] < floodfill[x][y]) {
    change_direction(1);
  }else if(coordinates[x][y][2] == 0 && x < maze_size-1 && floodfill[x+1][y] < floodfill[x][y]) {
    change_direction(2);
  }else if(coordinates[x][y][3] == 0 && y > 0 && floodfill[x][y-1] < floodfill[x][y]) {
    change_direction(3);
  }else if(coordinates[x][y][0] == 0 && x > 0 && floodfill[x-1][y] < floodfill[x][y]) {
    change_direction(0);
  }
  */

  //Ưu tiên rẽ phải -> đi thẳng -> rẽ trái -> lùi
  // if(coordinates[x][y][2] == 0 && x < maze_size-1 && floodfill[x+1][y] < floodfill[x][y]) {
  //   change_direction(2);
  // }else if(coordinates[x][y][1] == 0 && y < maze_size-1 && floodfill[x][y+1] < floodfill[x][y]) {
  //   change_direction(1);
  // }else if(coordinates[x][y][0] == 0 && x > 0 && floodfill[x-1][y] < floodfill[x][y]) {
  //   change_direction(0);
  // }else if(coordinates[x][y][3] == 0 && y > 0 && floodfill[x][y-1] < floodfill[x][y]) {
  //   change_direction(3);
  // }

  for(int i=0; i<4; i++) {
    switch(priorityDirection[i]) {
      case 0:
        if(coordinates[x][y][0] == 0 && x > 0 && floodfill[x-1][y] < floodfill[x][y]) {
          change_direction(0);
          i = 4;
        }
        break;
      case 1:
        if(coordinates[x][y][1] == 0 && y < maze_size-1 && floodfill[x][y+1] < floodfill[x][y]) {
          change_direction(1);
          i = 4;
        }
        
        break;
      case 2:
        if(coordinates[x][y][2] == 0 && x < maze_size-1 && floodfill[x+1][y] < floodfill[x][y]) {
          change_direction(2);
          i = 4;
        }
        break;
      case 3:
        if(coordinates[x][y][3] == 0 && y > 0 && floodfill[x][y-1] < floodfill[x][y]) {
          change_direction(3);
          i = 4;
        }
        break;
    }
  }
  

  moveForward();
  
}
