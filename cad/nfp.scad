length = 37;
width = 21;
height = 13;
module form() {
    resize([length,width,height]) minkowski() {
        resize([length,width,height]) sphere(1, $fn=30);
        sphere(3, $fn=15);
    }
    translate([-50,0,0]) rotate([0,90,0]) cylinder(d=6,h=40, $fn=10);
    translate([-30,0,0]) rotate([0,90,0]) cylinder(d=7,h=2, $fn=10);
    translate([-25,0,0]) rotate([0,90,0]) cylinder(d=7,h=2, $fn=10);
    translate([-35,0,0]) rotate([0,90,0]) cylinder(d=7,h=2, $fn=10);
    translate([-40,0,0]) rotate([0,90,0]) cylinder(d=7,h=2, $fn=10);
    translate([-20,0,0]) rotate([0,90,0]) cylinder(d=7,h=2, $fn=10);
    translate([-45,0,0]) rotate([0,90,0]) cylinder(d=7,h=2, $fn=10);
    translate([-50,0,0]) rotate([0,90,0]) cylinder(d=7,h=2, $fn=10);
}
difference() {
    translate([-53, -15, -24]) cube([75,30,24]);
    form();
    //translate([-12.5, -7.5, -43]) cube([25,15,40]);
    translate([-20, -12, -50]) cylinder(d=4,h=100, $fn=10);
    translate([-20, 12, -50]) cylinder(d=4,h=100, $fn=10);
    translate([18, 12, -50]) cylinder(d=4,h=100, $fn=10);
    translate([18, -12, -50]) cylinder(d=4,h=100, $fn=10);
    translate([-45, 12, -50]) cylinder(d=4,h=100, $fn=10);
    translate([-45, -12, -50]) cylinder(d=4,h=100, $fn=10);
}

translate([10, 15, -21]) cube([5, 5, 15]);
translate([-18, 15, -21]) cube([5, 5, 15]);
translate([10, -20, -21]) cube([5, 5, 15]);
translate([-18, -20, -21]) cube([5, 5, 15]);
translate([22, -20, -21]) cube([10, 40, 15]);
translate([-63, -20, -21]) cube([10, 40, 15]);
//rotate([180, 0, 0]) difference() {
//    translate([-35, -15, 0]) cube([60,30,3]);
//    form();
//}

// form();
 //color("grey") cube([25,14,7], center=true);

