/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose, 
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package com.bulletphysics.demos.basic;

import static com.bulletphysics.demos.opengl.IGL.GL_COLOR_BUFFER_BIT;
import static com.bulletphysics.demos.opengl.IGL.GL_DEPTH_BUFFER_BIT;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Vector3f;

import org.lwjgl.LWJGLException;

import com.bulletphysics.collision.broadphase.AxisSweep3;
import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.SphereShape;
import com.bulletphysics.collision.shapes.StaticPlaneShape;
import com.bulletphysics.demos.opengl.DemoApplication;
import com.bulletphysics.demos.opengl.GLDebugDrawer;
import com.bulletphysics.demos.opengl.IGL;
import com.bulletphysics.demos.opengl.LWJGL;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.SimpleDynamicsWorld;
import com.bulletphysics.dynamics.constraintsolver.ConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.Point2PointConstraint;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;

public class BasicDemo extends DemoApplication {
	//??????????????????
	private static final int ARRAY_SIZE_X = 1;
	private static final int ARRAY_SIZE_Y = 1;
	private static final int ARRAY_SIZE_Z = 1;
	//???????????????????????????????????????????????????
	private static final int MAX_PROXIES = (ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z + 1024);

	private static final int START_POS_X = -10;
	private static final int START_POS_Y = 0;
	private static final int START_POS_Z = -10;
	
	//?????????????????????????????????/??????
	private List<CollisionShape> collisionShapes = new ArrayList<CollisionShape>();
	private BroadphaseInterface overlappingPairCache;
	private CollisionDispatcher dispatcher;
	private ConstraintSolver solver;
	private DefaultCollisionConfiguration collisionConfiguration;
	int i=0;  //
	   int k=0;  //?????????????????????
	   long start=0;
		long end=0;
		Vector3f qishi=new Vector3f(0, 0, 0);
		double maxjuli=0;  //?????????????????????
		Vector3f zhigao=new Vector3f(0, 0, 0);
		Vector3f vbian=new Vector3f(0, 0, 0);

	public BasicDemo(IGL gl) {
		super(gl);
	}
	
	@Override
	public void clientMoveAndDisplay() {
		gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
		// simple dynamics world doesn't handle fixed-time-stepping
		//??????????????????????????????????????????????????????
		//????????????????????????
		float ms = getDeltaTimeMicroseconds();
		if (dynamicsWorld != null) {
			//dynamicsWorld.stepSimulation(ms / 10000f);
			dynamicsWorld.stepSimulation(1f/60f);

			//??????????????????????????????
			dynamicsWorld.debugDrawWorld();
		}

			//??????????????????
			for (int j=dynamicsWorld.getNumCollisionObjects()-1; j>=0; j--)
			{
				CollisionObject obj = dynamicsWorld.getCollisionObjectArray().getQuick(j);
				RigidBody body = RigidBody.upcast(obj);
				//GLDebugDrawer glDebugDrawer = new GLDebugDrawer(null);
				//glDebugDrawer.drawLine(cameraPosition, cameraPosition, cameraPosition);
				if (body != null && body.getMotionState() != null) {
					Transform trans = new Transform();
					body.getMotionState().getWorldTransform(trans);
					if(j==1) {
						Vector3f link = new Vector3f(0, 0, 0);
						
						body.getLinearVelocity(link);
						//?????????????????????
						float Gx=1/body.getInvMass()*(link.x*link.x+link.y*link.y+link.z*link.z)/(trans.origin.x*trans.origin.x+trans.origin.y*trans.origin.y+trans.origin.z*trans.origin.z);
						//System.out.println(Math.pow(link.x*link.x+link.y*link.y+link.z*link.z,0.5));
						//??????????????????????????????GMm/r^2
						//double Ying=(Math.pow(6.67259, -11)*Math.pow(5.965,24)*300)/Math.pow((trans.origin.x*trans.origin.x+trans.origin.y*trans.origin.y+trans.origin.z*trans.origin.z), 1.5);
						//double Ying=(6.67259*5.967*1000*10000)/Math.pow((trans.origin.x*trans.origin.x+trans.origin.y*trans.origin.y+trans.origin.z*trans.origin.z), 1.5);
						//??????????????????
						double Ying=(6.67259*5.967*1000*10000)/Math.pow((trans.origin.x*trans.origin.x+trans.origin.y*trans.origin.y+trans.origin.z*trans.origin.z), 1.5);
						float Yxi=new Float(Ying);
						//????????????
						Vector3f Yli =new Vector3f(-Yxi*trans.origin.x,-Yxi*trans.origin.y,-Yxi*trans.origin.z);
						//System.out.println(Yli);
						//?????????????????????
						Vector3f Fxiang =new Vector3f(-Gx*trans.origin.x,-Gx*trans.origin.y,-Gx*trans.origin.z);

						//???????????????
						//body.applyCentralForce(Fxiang);
						//??????????????????
						body.applyCentralForce(Yli);
						
						//????????????
						i=i+1;
						System.out.println(i);
						if(i>=120000) {
							
							if(i<=122000) {
								if(i==120000) {
									 start=System.currentTimeMillis();
									  qishi = trans.origin;
//									  Vector3f bian = new Vector3f((float) (link.x*80/Math.pow((link.x*link.x+link.y*link.y+link.z*link.z), 0.5))
//												,(float) (link.y*80/Math.pow((link.x*link.x+link.y*link.y+link.z*link.z), 0.5)), 
//												(float) (link.z*80/Math.pow((link.x*link.x+link.y*link.y+link.z*link.z), 0.5)));
									  
									   Vector3f bian = new Vector3f(-777f,-190f,0f);
									    body.getLinearVelocity(link);
									    System.out.print("?????????????????????");
										System.out.println(link);
										System.out.print("??????????????????");
										System.out.print(trans.origin);
										System.out.print("????????????????????????");
										System.out.println(bian);
										body.applyCentralImpulse(bian);  
								}
								if(i==120001) {
									body.getLinearVelocity(link);
									vbian=link;
									
								}
								System.out.println("????????????????????????"+(vbian));
								

							}else {
								
								if(i==122000) {
									 end=System.currentTimeMillis();	
								}
								//System.out.println("?????????????????????????????????:" + "=" + (end-start));
								System.out.println("???????????????????????????"+(qishi));
								
							}
							if(i==120600) {
								 start=System.currentTimeMillis();
								  qishi = trans.origin;
//								  Vector3f bian = new Vector3f((float) (link.x*80/Math.pow((link.x*link.x+link.y*link.y+link.z*link.z), 0.5))
//											,(float) (link.y*80/Math.pow((link.x*link.x+link.y*link.y+link.z*link.z), 0.5)), 
//											(float) (link.z*80/Math.pow((link.x*link.x+link.y*link.y+link.z*link.z), 0.5)));
								  
					
								    body.getLinearVelocity(link);
								    System.out.print("?????????");
									System.out.println(link);
									System.out.print("????????????");
									System.out.print(trans.origin);  
							}
							
							//System.out.println(i);
						}
						
						
						double juli=Math.pow((trans.origin.x*trans.origin.x+trans.origin.y*trans.origin.y+trans.origin.z*trans.origin.z), 0.5);
						
						//System.out.println("??????xian??????"+(zhigao)+"?????????xyu??????"+(juli));
						if(juli>maxjuli) {
							maxjuli=juli;
							zhigao =trans.origin;
						}
						body.getLinearVelocity(link);
					    System.out.print("?????????");
						System.out.println(link);
						System.out.println("??????????????????"+(trans.origin)+"????????????"+(juli));
						System.out.println("??????????????????"+(zhigao)+"??????????????????"+(maxjuli));
					}
					
				}
			}
	
		//??????
		
		renderme();
		//glFlush();
		//glutSwapBuffers();
	}

	@Override
	public void displayCallback() {
		gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		renderme();
		
		// optional but useful: debug drawing to detect problems
		//????????????
		if (dynamicsWorld != null) {
			dynamicsWorld.debugDrawWorld();
			dynamicsWorld.stepSimulation(1.f / 60.f);
			//dynamicsWorld.
		}

		//glFlush();
		//glutSwapBuffers();
	}

	public void initPhysics() {
		setCameraDistance(5000f);

		// collision configuration contains default setup for memory, collision setup
		//??????????????????????????????????????????????????????

		collisionConfiguration = new DefaultCollisionConfiguration();

		// use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
		//??????????????????????????????????????????????????????????????????????????????????????????????????????Extras/BullettMultiThreaded???
		dispatcher = new CollisionDispatcher(collisionConfiguration);

		// the maximum size of the collision world. Make sure objects stay within these boundaries
		// TODO: AxisSweep3
		// Don't make the world AABB size too large, it will harm simulation quality and performance
		//???????????????????????????????????????????????????????????????
		Vector3f worldAabbMin = new Vector3f(-300000,-300000,-300000);
		Vector3f worldAabbMax = new Vector3f(300000,300000,300000);
		overlappingPairCache = new AxisSweep3(worldAabbMin,worldAabbMax,MAX_PROXIES);
		//overlappingPairCache = new SimpleBroadphase(MAX_PROXIES);

		// the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
		//?????????????????????
		SequentialImpulseConstraintSolver sol = new SequentialImpulseConstraintSolver();
		solver = sol;
		
		// TODO: needed for SimpleDynamicsWorld
		//sol.setSolverMode(sol.getSolverMode() & ~SolverMode.SOLVER_CACHE_FRIENDLY.getMask());
		
		dynamicsWorld = new DiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
		//dynamicsWorld = new SimpleDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);

		dynamicsWorld.setGravity(new Vector3f(0f, 0f, 0f));
		

		{
			//??????????????????
			CollisionShape sphereShape = new SphereShape(4072.59f); 
			 
			collisionShapes.add(sphereShape);
			
			Transform sphereTransform = new Transform();
			sphereTransform.setIdentity();
			sphereTransform.origin.set(0, 0, 0);
			
			
			float mass=0f;
			
			boolean isDynamic = (mass != 0f);

			Vector3f localInertia = new Vector3f(0, 0, 0);
			if (isDynamic) {
				sphereShape.calculateLocalInertia(mass, localInertia);
			}
			
			DefaultMotionState myMotionState = new DefaultMotionState(sphereTransform);
			RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, myMotionState, sphereShape, localInertia);
			RigidBody body = new RigidBody(rbInfo);
			
			
			dynamicsWorld.addRigidBody(body);
			
			
			//?????????????????????????????????
			//??????????????????
//			CollisionShape sphereShape3 = new SphereShape(5);
//			 
//			collisionShapes.add(sphereShape3);
//			
//			Transform sphereTransform3 = new Transform();
//			sphereTransform3.setIdentity();
//			sphereTransform3.origin.set(10, -8, 0);
//			
//			float mass3=2f;
//			
//			boolean isDynamic3 = (mass3 != 0f);
//
//			Vector3f localInertia3 = new Vector3f(0, 0, 0);
//			if (isDynamic3) {
//				sphereShape.calculateLocalInertia(mass3, localInertia3);
//			}
//			
//			DefaultMotionState myMotionState3 = new DefaultMotionState(sphereTransform3);
//			RigidBodyConstructionInfo rbInfo3 = new RigidBodyConstructionInfo(mass3, myMotionState3, sphereShape3, localInertia3);
//			RigidBody body2 = new RigidBody(rbInfo3);
//			
//			
//			dynamicsWorld.addRigidBody(body2);
//			
//			Vector3f link1 = new Vector3f(5, 0, 0);
//			Vector3f link2 = new Vector3f(-1.5f, 0, 0);
//			
//			
//			Point2PointConstraint(body,localInertia);
//			Point2PointConstraint(body,body2,link1,link2);
//			
			
		
			// create a few dynamic rigidbodies
			// Re-using the same collision is better for memory usage and performance
			//?????????????????????????????????????????????

			//CollisionShape colShape = new BoxShape(new Vector3f(3, 3, 3));
			CollisionShape colShape = new SphereShape(300f);
			collisionShapes.add(colShape);

			// Create Dynamic Objects
			//??????????????????
			Transform startTransform = new Transform();
			startTransform.setIdentity();

			float mass2 = 1000f;

			// rigidbody is dynamic if and only if mass is non zero, otherwise static
			//????????????
			boolean isDynamic2 = (mass2 != 0f);

			Vector3f localInertia2 = new Vector3f(0, 0, 0);
			if (isDynamic2) {
				colShape.calculateLocalInertia(mass2, localInertia2);
			}


			//??????????????????????????????????????????????????????????????????????????????

			for (int k = 0; k < ARRAY_SIZE_Y; k++) {
				for (int i = 0; i < ARRAY_SIZE_X; i++) {
					for (int j = 0; j < ARRAY_SIZE_Z; j++) {
						startTransform.origin.set(7871.95259254794f,0f,0f);
                                         
						// using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
						//???????????????
						
						
						DefaultMotionState myMotionState2 = new DefaultMotionState(startTransform);
						
						RigidBodyConstructionInfo rbInfo2 = new RigidBodyConstructionInfo(mass2, myMotionState2, colShape, localInertia2);
						RigidBody weix = new RigidBody(rbInfo2);
						weix.setFriction(0);
						//?????????????????????
						
						Vector3f speed = new Vector3f(0f,7.105864222159f,0);
						//weix.setInterpolationLinearVelocity(speed);
					
						dynamicsWorld.addRigidBody(weix);
						
						weix.setLinearVelocity(speed);
						//weix.setAngularVelocity(speed);
						
						
					}
				}
				
			}
		
		}

		//??????????????????????????????????????????????????????????????????
		//clientResetScene();
	}
	
	
	public static void main(String[] args) throws LWJGLException {
		
		BasicDemo ccdDemo = new BasicDemo(LWJGL.getGL());
		ccdDemo.initPhysics();
		ccdDemo.getDynamicsWorld().setDebugDrawer(new GLDebugDrawer(LWJGL.getGL()));
		ccdDemo.setCameraDistance(12000f);
		
		LWJGL.main(args, 2000, 1300, "Bullet Physics Demo", ccdDemo);
		
	}
	
	
	
}
