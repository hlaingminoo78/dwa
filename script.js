// -min and max translational velocity
const MAX_TVEL = 1; //1
const MIN_TVEL = -1; //-1
// -min and max rotational velocity
const MAX_RVEL = (50.0 * Math.PI) / 180.0; //50
const MIN_RVEL = -1 * MAX_RVEL; //-1
// -translational acceleration
const MAX_TACC = 1; //0.5
// -rotataional acceleration (50deg/ss) (convert to rad/ss)
const MAX_RACC = (50.0 * Math.PI) / 180.0; //50

// -an increment value to search in the range
const TVEL_RESOLUTION = 0.1; //0.1
const RVEL_RESOLUTION = (1 * Math.PI) / 180.0; //1

// -an increment value for time (time interval)
let TIME_DELTA = 0.4; //0.5
// -how much time the robot predict the next position in advance
let PREDICT_TIME = 10; //8

// -size of robot
let ROBOT_RADIUS = 6; //6

const GOALWEIGHT = 0.15; //0.15
const SPEEDWEIGHT = 1;
const OBSTACLEWEIGHT = 1;

let startNode, endNode, obstacles, obstacleCount;
let x;
let footPrint;
let alreadyFound;

const CANVAS_WIDTH = 500;
const CANVAS_HEIGHT = 500;

// -function to reset all variables
function reset() {
  alreadyFound = false;

  // -get obstacleCount from user input
  obstacleCount = parseInt(document.getElementById("objcount").value);

  // -create obstacles without overlapping
  obstacles = [];
  while (obstacles.length < obstacleCount) {
    let isCollided = false;
    let tempNode = new Node(
      floor(random(CANVAS_WIDTH)),
      floor(random(CANVAS_HEIGHT)),
      floor(random(5, 30)),
      color(0, 0, 0)
    );
    for (let j = 0; j < obstacles.length; j++) {
      if (tempNode.isCollide(obstacles[j])) {
        isCollided = true;
        break;
      }
    }
    if (!isCollided) obstacles.push(tempNode);
  }

  // -create startNode without overlapping with obstacles
  while (true) {
    let isCollided = false;
    let tempNode = new Node(
      floor(random(CANVAS_WIDTH)),
      floor(random(CANVAS_HEIGHT)),
      ROBOT_RADIUS,
      color(0, 0, 255)
    );
    for (let i = 0; i < obstacles.length; i++) {
      if (tempNode.isCollide(obstacles[i])) {
        isCollided = true;
        break;
      }
    }
    if (!isCollided) {
      startNode = tempNode;
      break;
    }
  }

  // -create endNode without overlapping with obstacles
  while (true) {
    let isCollided = false;
    let tempNode = new Node(
      floor(random(CANVAS_WIDTH)),
      floor(random(CANVAS_HEIGHT)),
      ROBOT_RADIUS,
      color(255, 0, 0)
    );
    for (let i = 0; i < obstacles.length; i++) {
      if (tempNode.isCollide(obstacles[i])) {
        isCollided = true;
        break;
      }
    }
    if (!isCollided) {
      endNode = tempNode;
      break;
    }
  }

  // -initial state
  // -current x, current y, heading(rad), current tv(m/s), current rv(rad/s)
  startNode.heading = PI / 8;
  startNode.tv = 0;
  startNode.rv = 0;

  footPrint = [];
  footPrint.push(startNode.copy());

  // -clear the background
  background(250);

  // -draw robot
  startNode.show();

  // -draw obstacles
  obstacles.forEach((elt) => {
    elt.show();
  });

  // -draw endNode
  endNode.show();

  // -only run the draw loop once
  noLoop();
}

function setup() {
  // -create p5 canvas
  const canvas = createCanvas(CANVAS_WIDTH, CANVAS_HEIGHT);
  canvas.parent("canvas_container");

  document.getElementById("start_button").onclick = () => {
    if (!alreadyFound) loop();
    else alert("You already reached the goal!");
  };
  document.getElementById("stop_button").onclick = () => noLoop();
  document.getElementById("restart_button").onclick = () => reset();
  document.getElementById("objcount").onchange = () => reset();

  reset();
}

function draw() {
  // -clear the background
  background(250);

  // -draw footPrint
  stroke(100);
  noFill();
  beginShape();
  for (let i = 0; i < footPrint.length; i++) {
    vertex(footPrint[i].x, footPrint[i].y);
  }
  endShape();

  // -draw robot
  startNode.show();

  // -draw obstacles
  obstacles.forEach((elt) => {
    elt.show();
  });

  // -draw endNode
  endNode.show();

  // -get the velocity range in dynamic window
  let dw = calculateDynamicWindow(startNode);

  // -get the trajectory from best (v,w)
  let [bestv, bestw, trajectory] = calculateBestTrajectory(startNode, dw);

  // -update robot position
  motion(startNode, bestv, bestw);

  footPrint.push(startNode.copy());

  // -draw predicted trajectory
  noFill();
  stroke(0, 0, 255);
  beginShape();
  for (let i = 0; i < trajectory.length; i++) {
    vertex(trajectory[i].x, trajectory[i].y);
  }
  endShape();

  // -if the robot reach endNode, then stop the loop
  if (startNode.isCollide(endNode)) {
    noLoop();
    console.log("Finish");
    alreadyFound = true;
  }
}

// -function to calculate array of velocity pairs in dynamic window
function calculateDynamicWindow(node) {
  // -(Vs) the space of possible velocities
  let Vs = [MIN_TVEL, MAX_TVEL, MIN_RVEL, MAX_RVEL];
  // -(Vd) the space where the robot can reach in next time interval with its acceleration
  // - V_final = V_initial + Acceleration * Time
  let Vd = [
    // -translational velocity if the robot drive with full deceleration
    node.tv - MAX_TACC * TIME_DELTA,
    // -translational velocity if the robot drive with full acceleration
    node.tv + MAX_TACC * TIME_DELTA,
    // -rotational velocity if the robot drive with full deceleration
    node.rv - MAX_RACC * TIME_DELTA,
    // -rotational velocity if the robot drive with full acceleration
    node.rv + MAX_RACC * TIME_DELTA,
  ];
  // -(Vw) the space of dynamic window
  // -Vw = Vs ∩ Vd (intersection of Vs & Vd)
  let Vw = [
    max(Vs[0], Vd[0]),
    min(Vs[1], Vd[1]),
    max(Vs[2], Vd[2]),
    min(Vs[3], Vd[3]),
  ];
  return Vw;
}

// -find the best trajectory(v,w) based on objective function
function calculateBestTrajectory(node, dw) {
  let bestv = 0;
  let bestw = 0;
  let final_value = 0;
  let best_trajectory = [];

  // -for all translational velocities
  for (let i = dw[0]; i <= dw[1]; i += TVEL_RESOLUTION) {
    // -for all rotational velocities
    for (let j = dw[2]; j <= dw[3]; j += RVEL_RESOLUTION) {
      // -calculate multiple (x, y, v, w) for next predicted time
      let trajectory = predictTrajectory(node, i, j);

      // -calculate heading for next predicted time
      let heading = calculateHeading(trajectory);

      // -calculate velocity for next predicted time
      let velocity = calculateVelocity(trajectory);

      // -calculate distance for next predicted time
      let distance = calculateDistance(trajectory);

      // -filter admissible velocities
      if (distance < 0) {
        continue;
      }

      // -normalize 3 value to the range between -1 and 1
      heading = map(heading, 0, PI, -1, 1);
      velocity = map(velocity, MIN_TVEL, MAX_TVEL, -1, 1);

      // -apply the weight
      heading = GOALWEIGHT * heading;
      velocity = SPEEDWEIGHT * velocity;
      distance = OBSTACLEWEIGHT * distance;

      // -objective function
      let current_value = heading + velocity + distance;

      // -update the (v,w) with hightest objective value
      if (final_value < current_value) {
        final_value = current_value;
        bestv = i;
        bestw = j;
        best_trajectory = trajectory;
      }
    }
  }
  return [bestv, bestw, best_trajectory];
}

// -get trajectory with (v,w) within predicted time
function predictTrajectory(node, v, w) {
  let trajectory = [node.copy()];
  let newNode = node.copy();
  let time = 0;
  while (time <= PREDICT_TIME) {
    motion(newNode, v, w);
    trajectory.push(newNode.copy());
    time += TIME_DELTA;
  }
  return trajectory;
}

// -general motion equation
function motion(node, v, w) {
  node.heading += w * TIME_DELTA;
  node.x += v * cos(node.heading) * TIME_DELTA;
  node.y += v * sin(node.heading) * TIME_DELTA;
  node.tv = v;
  node.rv = w;
}

// -angle difference between predicted position and endNode position
function calculateHeading(trajectory) {
  let dx = endNode.x - trajectory[trajectory.length - 1].x;
  let dy = endNode.y - trajectory[trajectory.length - 1].y;
  let ang = Math.atan2(dy, dx);
  let error_ang = ang - trajectory[trajectory.length - 1].heading;
  let error = abs(Math.atan2(sin(error_ang), cos(error_ang)));
  return PI - error;
}

// -get the fastest/greatest translational velocity from trajectorys
function calculateVelocity(trajectory) {
  return trajectory[trajectory.length - 1].tv;
}

// -distance from predicted position to obstacles
function calculateDistance(trajectory) {
  for (let i = 0; i < trajectory.length; i++) {
    for (let j = 0; j < obstacles.length; j++) {
      // -if one of the node in trajectories collides with one of the obstacles, then return -1
      if (trajectory[i].isCollide(obstacles[j])) {
        return -1;
      }
    }
  }
  // -return 1 if it does not collide any obstacles
  return 1;
}
