

package com.bulletphysics.demos.basic;

import com.bulletphysics.util.ObjectArrayList;
import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.DbvtBroadphase;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.demos.opengl.DemoApplication;
import com.bulletphysics.demos.opengl.GLDebugDrawer;
import com.bulletphysics.demos.opengl.IGL;
import com.bulletphysics.demos.opengl.LWJGL;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.constraintsolver.ConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;
import javax.vecmath.Vector3f;
import org.lwjgl.LWJGLException;
import static com.bulletphysics.demos.opengl.IGL.*;

/**
 * BasicDemo is good starting point for learning the code base and porting.
 * 
 * @author jezek2
 */
public class BasicDemo extends DemoApplication {

	// 创建125（5x5x5）个动态对象
	private static final int ARRAY_SIZE_X = 5;
	private static final int ARRAY_SIZE_Y = 5;
	private static final int ARRAY_SIZE_Z = 5;

	//物体的最大数量（并允许用户射出方块）
	private static final int MAX_PROXIES = (ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z + 1024);

	private static final int START_POS_X = -5;
	private static final int START_POS_Y = -5;
	private static final int START_POS_Z = -3;
	
	// 保留碰撞形状，用于删除/清理
	private ObjectArrayList<CollisionShape> collisionShapes = new ObjectArrayList<CollisionShape>();
	private BroadphaseInterface broadphase;
	private CollisionDispatcher dispatcher;
	private ConstraintSolver solver;
	private DefaultCollisionConfiguration collisionConfiguration;

	public BasicDemo(IGL gl) {
		super(gl);
	}
	
	@Override
	public void clientMoveAndDisplay() {
		gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// simple dynamics world无法处理固定时间步长
		float ms = getDeltaTimeMicroseconds();

		// 时间步进
		if (dynamicsWorld != null) {
			dynamicsWorld.stepSimulation(ms / 1000000f);
			// 可选但有用：调试图形
			dynamicsWorld.debugDrawWorld();
		}

		renderme();

		//glFlush();
		//glutSwapBuffers();
	}

	@Override
	public void displayCallback() {
		gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		renderme();

		// 可选但有用：调试图形以检测问题
		if (dynamicsWorld != null) {
			dynamicsWorld.debugDrawWorld();
		}

		//glFlush();
		//glutSwapBuffers();
	}

	public void initPhysics() {
		setCameraDistance(50f);

		// 冲突配置包含内存的默认设置、冲突设置
		collisionConfiguration = new DefaultCollisionConfiguration();

		//使用默认的冲突调度程序。对于并行处理，您可以使用不同的调度程序（请参阅Extras/BulletMultiThreaded）
		dispatcher = new CollisionDispatcher(collisionConfiguration);

		broadphase = new DbvtBroadphase();

		//默认约束解算器。对于并行处理，您可以使用不同的解算器（请参阅Extras/BulletMultiThreaded）
		SequentialImpulseConstraintSolver sol = new SequentialImpulseConstraintSolver();
		solver = sol;
		
		// TODO: needed for SimpleDynamicsWorld
		//sol.setSolverMode(sol.getSolverMode() & ~SolverMode.SOLVER_CACHE_FRIENDLY.getMask());
		
		dynamicsWorld = new DiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

		dynamicsWorld.setGravity(new Vector3f(0f, -10f, 0f));

		// create a few basic rigid bodies
		CollisionShape groundShape = new BoxShape(new Vector3f(50f, 50f, 50f));
		//CollisionShape groundShape = new StaticPlaneShape(new Vector3f(0, 1, 0), 50);

		collisionShapes.add(groundShape);

		Transform groundTransform = new Transform();
		groundTransform.setIdentity();
		groundTransform.origin.set(0, -56, 0);

		// 我们也可以使用DemoApplication：：localCreateRigidBody，但为了清楚起见，这里使用它：
		{
			float mass = 0f;

			// 刚体是动态的当且仅当质量非零，否则是静态的
			boolean isDynamic = (mass != 0f);

			Vector3f localInertia = new Vector3f(0, 0, 0);
			if (isDynamic) {
				groundShape.calculateLocalInertia(mass, localInertia);
			}

			// 建议使用motionstate，它提供插值功能，并且仅同步“活动”对象

			DefaultMotionState myMotionState = new DefaultMotionState(groundTransform);
			RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, myMotionState, groundShape, localInertia);
			RigidBody body = new RigidBody(rbInfo);

			// add the body to the dynamics world
			dynamicsWorld.addRigidBody(body);
		}

		{
			// 创建几个动态刚体
			//重复使用相同的冲突对内存使用和性能都有好处

			CollisionShape colShape = new BoxShape(new Vector3f(1, 1, 1));
			//CollisionShape colShape = new SphereShape(1f);
			collisionShapes.add(colShape);

			// 创建动态对象
			Transform startTransform = new Transform();
			startTransform.setIdentity();

			float mass = 1f;

			// 当且仅当刚体质量为零时其为静态
			boolean isDynamic = (mass != 0f);

			Vector3f localInertia = new Vector3f(0, 0, 0);
			if (isDynamic) {
				colShape.calculateLocalInertia(mass, localInertia);
			}

			float start_x = START_POS_X - ARRAY_SIZE_X / 2;
			float start_y = START_POS_Y;
			float start_z = START_POS_Z - ARRAY_SIZE_Z / 2;

			for (int k = 0; k < ARRAY_SIZE_Y; k++) {
				for (int i = 0; i < ARRAY_SIZE_X; i++) {
					for (int j = 0; j < ARRAY_SIZE_Z; j++) {
						startTransform.origin.set(
								2f * i + start_x,
								10f + 2f * k + start_y,
								2f * j + start_z);

						// 建议使用motionstate，它提供插值功能，并且仅同步“活动”对象
						DefaultMotionState myMotionState = new DefaultMotionState(startTransform);
						RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, myMotionState, colShape, localInertia);
						RigidBody body = new RigidBody(rbInfo);
						body.setActivationState(RigidBody.ISLAND_SLEEPING);

						dynamicsWorld.addRigidBody(body);
						body.setActivationState(RigidBody.ISLAND_SLEEPING);
					}
				}
			}
		}

		clientResetScene();
	}
	
	public static void main(String[] args) throws LWJGLException {
		BasicDemo ccdDemo = new BasicDemo(LWJGL.getGL());
		ccdDemo.initPhysics();
		ccdDemo.getDynamicsWorld().setDebugDrawer(new GLDebugDrawer(LWJGL.getGL()));

		LWJGL.main(args, 800, 600, "Bullet Physics Demo. http://bullet.sf.net", ccdDemo);
	}
	
}
