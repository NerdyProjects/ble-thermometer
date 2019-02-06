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
    translate([-55, -15, -10]) cube([80,30,10]);
    form();
    cube([25,15,30], center=true);
}
//translate([-35, 0, -10]) cylinder(d=2,h=20, $fn=10);
//rotate([180, 0, 0]) difference() {
//    translate([-35, -15, 0]) cube([60,30,3]);
//    form();
//}

// form();
 //color("grey") cube([25,14,7], center=true);

