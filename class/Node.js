class Node {
  constructor(x, y, r, c, heading = 0, tv = 0, rv = 0) {
    this.x = x; //x position
    this.y = y; //y position
    this.r = r; //radius
    this.c = c; //color
    this.heading = heading; //radian
    this.tv = tv; //translational velocity
    this.rv = rv; //rotational velocity
  }

  // -to display node
  show() {
    fill(this.c);
    noStroke();
    circle(this.x, this.y, this.r * 2);
  }

  // -function to check if the two object collide
  isCollide(node) {
    let distance = dist(this.x, this.y, node.x, node.y);
    // -if distance is shorter than combination of two radius, then they collide
    if (distance <= this.r + node.r) return true;
    return false;
  }

  // -return clone of this node
  copy() {
    return new Node(
      this.x,
      this.y,
      this.r,
      this.c,
      this.heading,
      this.tv,
      this.rv
    );
  }
}
