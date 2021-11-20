var transformY = function(y) {
    return -y + 400;
};
var transformAngle = function(a) {
    return -a;
};
var transformAngle2 = function(a) {
    return a > 0 ? (a > PI ? a - TWO_PI : a) : (a < -PI ? TWO_PI + a : a);
};
var rnd = function(x) {
    return round(x * 100) / 100;
};
var sign = function(x) {
    return x === 0 ? 0 : abs(x) / x;
};
var ty = transformY;
var ta = transformAngle;
var ta2 = transformAngle2;

angleMode = "radians";

var zero = millis();

var actionlog = [[]];
var targets = [[100, 50, 2, 1], [100, 200, 1, 0], [200, 350, 0, 0], [200, 200, 1, 0], [200, 50, 0, 0], [300, 200, 1, 0], [200, 350, 0, 0], [350, 50, 1, 0], [200, 350, 0, 0], [350, 350, 1, 1], [50, 350, 0, 0], [200, 100, 0, 0]]; //coordinates of target points, in the form [x, y, target speed]
var currenttarget = 0;

frameRate(100);

var Robot = function() {
    this.leftdrive = 0;
    this.rightdrive = 0;
    this.leftspeed = 0;
    this.rightspeed = 0;
    this.x = 50;
    this.y = 50;
    this.heading = PI;
    this.trackwidth = 30;
    this.target = {x: targets[0][0], y: targets[0][1], vel: targets[0][2], dir: targets[0][3]};
    this.kp = 0.9;
    this.normalkp = this.kp;
    this.boostkp = 2;
    this.boostkpthreshold = 0.5;
    this.maxspeed = 1;
    this.originalspeed = 0;
    this.originalpos = [this.x, this.y];
    this.actiontime = 0;
    this.distance = 0;
};

Robot.prototype.display = function() {
    rectMode(CENTER);
    pushMatrix();
    translate(this.x, ty(this.y));
    noStroke();
    rotate(ta(this.heading));
    fill(160, 160, 160, 255);
    rect(0, 0, 30, 30);
    fill(100, 100, 100, 255);
    rect(-8, 15, 10, 4);
    rect(8, 15, 10, 4);
    rect(-8, -15, 10, 4);
    rect(8, -15, 10, 4);
    strokeWeight(4);
    stroke(255, 0, 0);
    line(10, 0, 30, 0);
    popMatrix();
};
Robot.prototype.update = function() { //odometry
    this.leftspeed = constrain(this.leftspeed, -this.maxspeed, this.maxspeed);
    this.rightspeed = constrain(this.rightspeed, -this.maxspeed, this.maxspeed);
    this.leftdrive += this.leftspeed;
    this.rightdrive += this.rightspeed;
    this.netanglechange = (this.leftspeed - this.rightspeed) / this.trackwidth;
    if (this.netanglechange !== 0) {
        this.chorddistance = 2 * sin(this.netanglechange / 2) * (this.rightspeed / this.netanglechange + this.trackwidth / 2);
    } else {
        this.chorddistance = sqrt(this.leftspeed * this.leftspeed + this.rightspeed * this.rightspeed);
    }
    if (this.netanglechange !== 0) {
        this.x += this.chorddistance * cos(this.netanglechange + this.heading);
        this.y += this.chorddistance * sin(this.netanglechange + this.heading);
    } else {
        this.x += this.rightspeed * cos(this.heading);
        this.y += this.rightspeed * sin(this.heading);
    }
    this.heading += ta(this.netanglechange);
    this.heading = ta2(this.heading % TWO_PI);
};
Robot.prototype.calculateSpeed = function() { //speed adjustment to save time in longer movement
    this.x_ = dist(this.originalpos[0], this.originalpos[1], this.x, this.y) / dist(this.originalpos[0], this.originalpos[1], this.target.x, this.target.y);
    var k0 = constrain(this.originalspeed, 0, 3);
    var k1 = this.target.vel;
    var n = min(k0, k1) / max(k0, k1);
    n = n / (n + 1);
    var p = k0 * k0 + k1 * k1;
    
    var A = (p - k0 + n * (k0 - k1)) / (n * (n - 1));
    var B = (k0 - p + n * n * (k1 - k0)) / (n * (n - 1));
    
    this.adjustedspeed = min(A * this.x_ * this.x_ + B * this.x_ + k0, this.maxspeed);
};
Robot.prototype.moveToTarget = function() { //moving towards target
    if (this.target.x !== null) {
        if (abs(this.differenceangle) > this.boostkpthreshold) {
            this.kp = this.boostkp;
        } else {
            this.kp = this.normalkp;
        }
        if (this.target.dir === 0) {
            this.differencex = this.target.x - this.x;
            this.differencey = this.target.y - this.y;
            this.differenceangle = ta2(atan2(this.differencey, this.differencex) - this.heading);
            this.distanceprev = this.distance;
            this.distance = sqrt(this.differencex * this.differencex + this.differencey * this.differencey);
            this.distancederivative = this.distance - this.distanceprev;
            this.leftspeed = this.target.vel - this.differenceangle * this.kp;
            this.rightspeed = this.target.vel + this.differenceangle * this.kp;
            if (abs(this.differenceangle) < 0.1) {
                this.calculateSpeed();
                this.leftspeed = this.adjustedspeed - this.differenceangle * this.kp;
                this.rightspeed = this.adjustedspeed + this.differenceangle * this.kp;
            }
            if ((abs(this.differencex) < 3 && abs(this.differencey) < 3) || (abs(this.distancederivative) < 0.0001 && this.x_ > 0.9 && this.target.vel > 0)) {
                this.originalspeed = (this.leftspeed + this.rightspeed) / 2;
                this.originalpos = [this.x, this.y];
                currenttarget++;
                    //this.target = {x: random(50, 350), y: random(50, 350), vel: random(0.5, 2)};
                this.setTarget(targets[currenttarget][0], targets[currenttarget][1], targets[currenttarget][2], targets[currenttarget][3]);
                this.actiontime = millis() - this.actiontime;
                actionlog.unshift([this.differencex, this.differencey, this.actiontime]);
                this.actiontime = millis();
                this.x_ = dist(this.originalpos[0], this.originalpos[1], this.x, this.y) / dist(this.originalpos[0], this.originalpos[1], this.target.x, this.target.y);
            } 
        } else {
            this.differencex = this.target.x - this.x;
            this.differencey = this.target.y - this.y;
            this.differenceangle = ta2(PI + ta2(atan2(this.differencey, this.differencex) - this.heading));
            this.distanceprev = this.distance;
            this.distance = sqrt(this.differencex * this.differencex + this.differencey * this.differencey);
            this.distancederivative = this.distance - this.distanceprev;
            this.leftspeed = -this.target.vel - this.differenceangle * this.kp;
            this.rightspeed = -this.target.vel + this.differenceangle * this.kp;
            if (abs(this.differenceangle) < 0.1) {
                this.calculateSpeed();
                this.leftspeed = -this.adjustedspeed + this.differenceangle * this.kp;
                this.rightspeed = -this.adjustedspeed - this.differenceangle * this.kp;
            }
            if ((abs(this.differencex) < 3 && abs(this.differencey) < 3) || (abs(this.distancederivative) < 0.0001 && this.x_ > 0.9 && this.target.vel > 0)) {
                this.originalspeed = (this.leftspeed + this.rightspeed) / 2;
                this.originalpos = [this.x, this.y];
                this.actiontime = millis() - this.actiontime;
                actionlog.unshift([this.differencex, this.differencey, this.actiontime]);
                this.actiontime = millis();currenttarget++;
                //this.target = {x: random(50, 350), y: random(50, 350), vel: random(0.5, 2)};
                this.setTarget(targets[currenttarget][0], targets[currenttarget][1], targets[currenttarget][2], targets[currenttarget][3]);
                this.x_ = dist(this.originalpos[0], this.originalpos[1], this.x, this.y) / dist(this.originalpos[0], this.originalpos[1], this.target.x, this.target.y);
            }
        }
    }
};
Robot.prototype.setTarget = function(x, y, vel, dir) {
    this.target.x = x;
    this.target.y = y;
    this.target.vel = vel;
    this.target.dir = dir;
};

var r = new Robot();

var cullactionlog = function(threshold) {
    if (actionlog[0][2] < threshold) {
        actionlog.shift();
    }
};
var drawBackground = function() {
    background(0);
    textSize(13);
    textAlign(CENTER, CENTER);
    strokeWeight(1);
    for (var i = 1; i < 8; i++) {
        stroke(200, 200, 200, 100);
        line(i * 50, 0, i * 50, 400);
        line(0, i * 50, 400, i * 50);
    }
    noStroke();
    for (var i = 1; i < 8; i++) {
        fill(0);
        rect(i * 50 - 0, 390, 26, 14);
        rect(15, i * 50, 15, 24);
        fill(255, 255, 255, 255);
        text(i * 50, i * 50, 390);
        pushMatrix();
        translate(15, i * 50);
        rotate(PI / 2);
        text(400 - i * 50, 0, 0);
        popMatrix();
    }
    strokeWeight(10);
    stroke(0, 150, 0);
    point(r.target.x, ty(r.target.y));
};
var hud = function() {
    strokeWeight(10);
    stroke(127, 127, 127, 255);
    line(-20, 420, 420, 420);
    fill(255);
    textSize(15);
    textAlign(LEFT, CENTER);
    text("Position: " + rnd(r.x) + ", " + rnd(r.y), 13, 450);
    text("Encoders: " + rnd(r.leftdrive) + ", " + rnd(r.rightdrive), 13, 469);
    text("Velocity: " + rnd(r.leftspeed) + ", " + rnd(r.rightspeed), 13, 488);
    text("Heading: " + rnd(r.heading) + " rad, " + rnd(r.heading * 180 / PI) + " deg", 13, 507);
    text("Target: " + (r.target.x !== null ? (rnd(r.target.x) + ", " + rnd(r.target.y)) : "No target set"), 13, 536);
    text("Difference angle: " + (r.target.x !== null ? (rnd(r.differenceangle) + " rad, " + rnd(r.differenceangle * 180 / PI) + " deg") : "No target set"), 13, 555);
    text ("Difference: " + (r.target.x !== null ? (rnd(r.differencex) + ", " + rnd(r.differencey)) : "No target set"), 13, 574);
    text("Time: " + (millis() - zero) + "ms", 263, 450);
    text("Most recent\nmovements: ", 263, 479);
    cullactionlog(20);
    for (var i = 0; i < min(5, actionlog.length); i++) {
        text(rnd(actionlog[i][0]) + ", " + rnd(actionlog[i][1]) + ", " + rnd(actionlog[i][2]) + "ms", 263, 511 + 19 * i);
    }
};

draw = function() {
    drawBackground();
    hud();
    r.display();
    r.update();
    r.moveToTarget();
};

var paused = false;

mouseClicked = function() {
    if (paused) {
        loop();
        paused = false;
    } else {
        noLoop();
        paused = true;
    }
};
