$fn=40;

box(55,65,28,2);

//translate([0,0,40])lid(55,70,2);

module box(width, depth, height,wall){
difference(){
    hull(){
        translate([0,depth,0])cylinder(r=wall,h=height+wall);
        translate([width,depth,0])cylinder(r=wall,h=height+wall);
        translate([width,0,0])cylinder(r=wall,h=height+wall);
        translate([0,0,0])cylinder(r=wall,h=height+wall);
    }
    translate([0,0,wall])cube([width,depth,height+wall]);
    
    translate([42,50,15])rotate([-90,0,0])cylinder(d=21,h=30);
    translate([2,50,5])cube([10,30,20]);
    
    translate([12.5,-5,6])cube([30,30,18]);
    
    translate([49,-5,9])rotate([-90,0,0])cylinder(d=6,h=30);
    translate([49,-5,21])rotate([-90,0,0])cylinder(d=6,h=30);
    translate([6.5,-5,15])rotate([-90,0,0])cylinder(d=6,h=30);
    translate([-5,28,8])cube([20,4,12]);
    translate([-5,12,12])cube([20,4,8]);
    
    translate([25,35,15])rotate([0,90,0])cylinder(d=7.5,h=50);
}
}


module lid(width, depth,wall){
difference(){
    hull(){
        translate([0,depth,0])cylinder(r=wall,h=wall);
        translate([width,depth,0])cylinder(r=wall,h=wall);
        translate([width,0,0])cylinder(r=wall,h=wall);
        translate([0,0,0])cylinder(r=wall,h=wall);
    }
}
difference(){
translate([0,0,-wall])cube([width,depth,wall]);
    translate([wall,0,-wall])cube([width-2*wall,depth,wall]);
}
}