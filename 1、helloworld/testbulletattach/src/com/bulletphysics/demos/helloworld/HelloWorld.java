/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 * HelloWorld port by: Clark Dorman
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

package com.bulletphysics.demos.helloworld;

import com.bulletphysics.collision.broadphase.AxisSweep3;
import com.bulletphysics.collision.dispatch.CollisionConfiguration;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.SphereShape;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.ObjectArrayList;
import javax.vecmath.Vector3f;

/**
 * This is a Hello World program for running a basic Bullet physics simulation.
 * it is a direct translation of the C++ HelloWorld app.
 *
 * @author cdorman
 */
public class HelloWorld
{
	public static void main(String[] args) {
		// 碰撞配置包含内存的默认设置，碰撞设置。高级用户可以创建自己的配置。一般不用更改
		CollisionConfiguration collisionConfiguration = new DefaultCollisionConfiguration();

		// 使用默认的冲突调度程序。对于并行处理，您//可以使用不同的调度程序（请参阅Extras/BulletMultiThreaded）
		CollisionDispatcher dispatcher = new CollisionDispatcher(
				collisionConfiguration);

		// 此处对仿真世界的大小进行固定，在常规仿真中世界边界不应设置过大，过大就会降低仿真精确度。
		Vector3f worldAabbMin = new Vector3f(-10000, -10000, -10000);
		Vector3f worldAabbMax = new Vector3f(10000, 10000, 10000);
		int maxProxies = 1024;
		AxisSweep3 overlappingPairCache =
				new AxisSweep3(worldAabbMin, worldAabbMax, maxProxies);

		// 默认约束解算器，此处一般采用默认设置。对于并行处理，您可以使用不同的解算器（请参阅Extras/BulletMultiThreaded）
		SequentialImpulseConstraintSolver solver = new SequentialImpulseConstraintSolver();

		DiscreteDynamicsWorld dynamicsWorld = new DiscreteDynamicsWorld(
				dispatcher, overlappingPairCache, solver,
				collisionConfiguration);
        //设置仿真世界的重力。
		dynamicsWorld.setGravity(new Vector3f(0, -10, 0));

		// 创建仿真基础刚体，创建地面
		CollisionShape groundShape = new BoxShape(new Vector3f(50.f, 50.f, 50.f));

		//跟踪形状，我们在退出时释放内存。
		//确保在刚体中重复使用碰撞形状！
		ObjectArrayList<CollisionShape> collisionShapes = new ObjectArrayList<CollisionShape>();

		collisionShapes.add(groundShape);
         
		Transform groundTransform = new Transform();
		groundTransform.setIdentity();
		groundTransform.origin.set(new Vector3f(0.f, -56.f, 0.f));

		{
			float mass = 0f;

			// 当且仅当质量非零时，刚体是动态的，否则是静态的，我们可以通过创建质量为0的物体来实现固定面。
			boolean isDynamic = (mass != 0f);

			Vector3f localInertia = new Vector3f(0, 0, 0);
			if (isDynamic) {
				groundShape.calculateLocalInertia(mass, localInertia);
			}

			// 建议使用motionstate，它提供插值功能，并且仅同步“活动”对象
			DefaultMotionState myMotionState = new DefaultMotionState(groundTransform);
			RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(
					mass, myMotionState, groundShape, localInertia);
			RigidBody body = new RigidBody(rbInfo);

			// 将实体添加到动力学世界
			dynamicsWorld.addRigidBody(body);
		}

		{
			// 创建动态刚体

			CollisionShape colShape = new SphereShape(1.f);
			collisionShapes.add(colShape);

			// 创建动态对象
			Transform startTransform = new Transform();
			startTransform.setIdentity();

			float mass = 1f;

			// 当且仅当质量非零时，刚体是动态的，否则是静态的
			boolean isDynamic = (mass != 0f);

			Vector3f localInertia = new Vector3f(0, 0, 0);
			if (isDynamic) {
				colShape.calculateLocalInertia(mass, localInertia);
			}

			startTransform.origin.set(new Vector3f(2, 10, 0));

			// 建议使用motionstate，它提供//插值功能，并且仅同步//“活动”对象
			DefaultMotionState myMotionState = new DefaultMotionState(startTransform);

			RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(
					mass, myMotionState, colShape, localInertia);
			RigidBody body = new RigidBody(rbInfo);

			dynamicsWorld.addRigidBody(body);
		}

		//仿真模拟
		for (int i=0; i<100; i++) {
			dynamicsWorld.stepSimulation(1.f / 60.f, 10);

			// 打印所有对象的位置

			for (int j=dynamicsWorld.getNumCollisionObjects()-1; j>=0; j--)
			{
				CollisionObject obj = dynamicsWorld.getCollisionObjectArray().getQuick(j);
				RigidBody body = RigidBody.upcast(obj);
				if (body != null && body.getMotionState() != null) {
					Transform trans = new Transform();
					body.getMotionState().getWorldTransform(trans);
					System.out.printf("world pos = %f,%f,%f\n", trans.origin.x,
							trans.origin.y, trans.origin.z);
				}
			}
		}
	}
}
