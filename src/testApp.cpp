#include "testApp.h"

const int Nx =400;
const int Ny =300;


const int Ncir =1400;
ofVec2f Center;
vector<bubble> VecofBubbles;
float Tc = 0.1; // threshold of clossenes for collision
float Kcol = 0.6; //Scale factor for the collision repulsion
float Ka = 1.2; // scale factor for velocity attraction to center
float dam =3.0;
float Kb = 1.2;// scale factor for position attraction to center

float Krad =0.05;
cv::Mat  GrayCV;

// Minimun radious
float MinR = 1.0;
// Maximun radious
float MaxR = 10;
float Alpha = 1.62;

// TODO


// no linear radio, velocity aceleration damping en radio
// face detection
// logica para creacion eliminacion
// revisar locura cuando el radio es 1


//--------------------------------------------------------------
void testApp::setup(){
    
//    vidGrabber.setVerbose(true);
    vidGrabber.initGrabber(Nx,Ny);
    colorImg.allocate(Nx,Ny);
	grayImage.allocate(Nx,Ny);
//     ToRead.loadImage("cuadros.png");
    
    
    // first circles in random  positions
    Center.set(Nx/2.0,Ny/2.0);
    for (int k=0; k<Ncir; k++) {
        float x = ofRandom(1*Nx/8.0, 7*Nx/8.0);
        float y = ofRandom(1*Ny/8.0, 7*Ny/8.0);
        float rad = ofRandom(MinR,MaxR);
        float dc = sqrt((x - Center.x)*(x - Center.x) +
                        (y - Center.y)*(y - Center.y));
        BubbleQueue.push(bubble(x, y, rad, dc));
    }
    

    
    
       
}

//--------------------------------------------------------------
void testApp::update(){
    bool bNewFrame = false;
    
    vidGrabber.update();
      bNewFrame = vidGrabber.isFrameNew();
    //
      if (bNewFrame){
    //
    //
    colorImg.setFromPixels(vidGrabber.getPixels(), Nx,Ny);
    grayImage = colorImg;
    
    GrayCV = grayImage.getCvImage();
    
    //
    //    }
    // Geting the integral image;
    cv::Mat TheIntegral;
    cv::integral(GrayCV, TheIntegral,CV_32F);
    
    // copy al the sorted queue to a vector
    
    VecofBubbles.clear();
    while(!BubbleQueue.empty()) {
        bubble tempbub;
        tempbub = BubbleQueue.top();
        VecofBubbles.push_back(tempbub);
        BubbleQueue.pop();
    }
    
    // changing size loop
    // Checking the gray area behind
    for (int b1 = 0; b1< Ncir; b1++) {
      
        
        if((VecofBubbles[b1].x>=0)&&(VecofBubbles[b1].x<TheIntegral.cols)
         &&(VecofBubbles[b1].y>=0)&&(VecofBubbles[b1].y<TheIntegral.rows))
        {
        
        int x1 = MAX(0,VecofBubbles[b1].x - VecofBubbles[b1].rad);
            x1 = MIN(TheIntegral.cols-1,x1);
        
        int x2 = MIN(TheIntegral.cols-1,VecofBubbles[b1].x + VecofBubbles[b1].rad);
            x2 = MAX(0,x2);
        
        int y1 = MAX(0,VecofBubbles[b1].y - VecofBubbles[b1].rad);
            y1 = MIN(TheIntegral.rows-1,y1);
        
        int y2 = MIN(TheIntegral.rows-1,VecofBubbles[b1].y + VecofBubbles[b1].rad);
            y2 = MAX(0,y2);
        // The sum on the integral image:

//            cout<<x1<<","<<x2<<","<<y1<<","<<y2<<","<<VecofBubbles[b1].x<<","<<VecofBubbles[b1].y<<endl;
//            cout<<VecofBubbles[b1].rad<<endl;
        float TheSum = TheIntegral.at<float>(y2,x2)
                       - TheIntegral.at<float>(y2,x1)
                       - TheIntegral.at<float>(y1,x2)
                       + TheIntegral.at<float>(y1,x1);
        // number of pixels
        float Npix = (x2-x1)*(y2-y1);
        
        //Average
        float TheAv = 0;
        if (Npix !=0){
            TheAv = TheSum/Npix;
        }
        
        float TargRad = MinR + (MaxR-MinR)*powf(TheAv/255.0,Alpha);
        
        VecofBubbles[b1].rad += Krad*(TargRad -VecofBubbles[b1].rad);
            if(VecofBubbles[b1].rad<MinR){VecofBubbles[b1].rad=MinR;}
            if(VecofBubbles[b1].rad>MaxR){VecofBubbles[b1].rad=MaxR;}
        }
        
    }
    
    
    
    
    // The closest to the center is now on position 0
    
    // collision test from center to periphery
    
    for (int b1 = 0; b1< Ncir; b1++) {
        
        for (int b2 = b1+1; b2< Ncir; b2++) {
            
            // distance between bubbles
            
            float dx = VecofBubbles[b2].x - VecofBubbles[b1].x;
            float dy = VecofBubbles[b2].y - VecofBubbles[b1].y;
            ofVec2f difs(dx,dy);
            
            float d = difs.length();
            float r = VecofBubbles[b2].rad + VecofBubbles[b1].rad;
            
            // if the distance is less than the summ of the radii
            // move b2 away
            if((d+Tc)<r){
            
             difs.normalize();
             
             // directly alter position
           // to DO : use force to calculate acc then integrate to vel and then position?
            //  or it doesn make sence since on every inner loop each bubble can ve moved
                // maybe if the inner loop is inverted(ref is the one that is going to be moved)
                
                VecofBubbles[b2].x += Kcol*difs.x;
                VecofBubbles[b2].y += Kcol*difs.y;
            }
         }
 
        
    }
    
 
    // atraction loop
    
    
   for (int b1 = 0; b1< Ncir; b1++) {  
    
       float dxc = Center.x - VecofBubbles[b1].x;
       float dyc = Center.y - VecofBubbles[b1].y;
       ofVec2f difsc(dxc,dyc);
       difsc.normalize();
       
       // update velocity
       // Rad Scale fac
       
       float RadScale = ((VecofBubbles[b1].rad+1)*(VecofBubbles[b1].rad+1));
       VecofBubbles[b1].velx=Ka*difsc.x/(RadScale)
                            + VecofBubbles[b1].velx*(1 - dam/(RadScale));
       VecofBubbles[b1].vely=Ka*difsc.y/(RadScale)
       + VecofBubbles[b1].vely*(1 - dam/(RadScale));
       
       // update position
       
       VecofBubbles[b1].x += Kb*VecofBubbles[b1].velx;
       VecofBubbles[b1].y += Kb*VecofBubbles[b1].vely;
       
       BubbleQueue.push(bubble(VecofBubbles[b1].x,
                            VecofBubbles[b1].y,
                            VecofBubbles[b1].rad,
                            VecofBubbles[b1].velx,
                            VecofBubbles[b1].vely,
                            Center));
   }
    


      }
    
    
    
    

    
}




//--------------------------------------------------------------
void testApp::draw(){
	ofSetHexColor(0xffffff);
//    grayImage.draw(0,0);
    grayImage.draw(0,0);
  // Plot circles
    int k =0;
    ofNoFill();
    ofSetColor(0, 0, 0);
    for( int k = 0; k< VecofBubbles.size();k++) {  
        bubble tempbub;
        tempbub = VecofBubbles[k];
        ofCircle(400+2*tempbub.x, 2*tempbub.y, 2*tempbub.rad);

    }
    
    
    
    
    
}












//--------------------------------------------------------------
void testApp::keyPressed(int key){
    
    switch (key) {
        case 'r': // RESET
            VecofBubbles.clear();
            while(!BubbleQueue.empty()) {
                BubbleQueue.pop();
            }
            for (int k=0; k<Ncir; k++) {
                float x = ofRandom(1*Nx/8.0, 7*Nx/8.0);
                float y = ofRandom(1*Ny/8.0, 7*Ny/8.0);
                float rad = ofRandom(4,20);
                float dc = sqrt((x - Center.x)*(x - Center.x) +
                                (y - Center.y)*(y - Center.y));
                BubbleQueue.push(bubble(x, y, rad, dc));
            }
            
            

            break;
            
        case 's':
      
            break;
        case OF_KEY_RIGHT:
            dam+=0.2;
            cout<<dam<<endl;
            break;
        case OF_KEY_LEFT:
            dam-=.2;
            cout<<dam<<endl;
            break;
        case 'e':
            break;
            
        case 'a':
            Alpha+=.02;
            cout<<Alpha<<endl;
            break;
        case 'z':
            Alpha-=.02;
            cout<<Alpha<<endl;
            break;
        default:
            break;
    }
    
    
}

//--------------------------------------------------------------
void testApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y){
    
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){
    
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){
    
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo){ 
    
}