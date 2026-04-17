package org.firstinspires.ftc.teamcode.common.hardwares.Gamepad;

import org.firstinspires.ftc.teamcode.common.Robot;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.function.Consumer;

public class GamepadReflectBinder {

    /**
     * 必须在robot中调用，会扫描所有绑定在Robot中的子系统和OpMode中绑定的按键事件，如果没有在Robot中绑定则不会扫描
     * @param robot 要扫描的Robot类
     */
    public static void bind(Robot robot) {
        if (robot.gamepad1 == null || robot.gamepad2 == null || robot.opMode == null) {
            throw new IllegalStateException("Please call robot.init() before bind()，make sure gamepad and opMode has been initialized！");
        }

        // 获取当前正在运行的 OpMode 的类
        Class<?> currentOpModeClass = robot.opMode.getClass();

        // 1. 扫描 Robot 本身
        scanAndBindTree(robot, robot.gamepad1, robot.gamepad2, currentOpModeClass);

        // 2. 扫描所有子系统变量
        Field[] fields = robot.getClass().getDeclaredFields();
        for (Field field : fields) {
            field.setAccessible(true);
            try {
                Object subsystem = field.get(robot);
                if (subsystem != null && subsystem.getClass().getName().startsWith("org.firstinspires.ftc.teamcode")) {
                    scanAndBindTree(subsystem, robot.gamepad1, robot.gamepad2, currentOpModeClass);
                }
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }
    }

    public static void scanAndBindTree(Object target, GamepadEx gp1, GamepadEx gp2, Class<?> currentOpModeClass) {
        Class<?> clazz = target.getClass();
        while (clazz != null && clazz != Object.class) {
            scanAndBind(target, clazz, gp1, gp2, currentOpModeClass);
            clazz = clazz.getSuperclass(); // 向上遍历父类
        }
    }

    private static void scanAndBind(Object target, Class<?> clazz, GamepadEx gp1, GamepadEx gp2, Class<?> currentOpModeClass) {

        Method[] methods = clazz.getDeclaredMethods();

        for (Method method : methods) {
            if (method.isAnnotationPresent(GamepadBind.class)) {
                GamepadBind annotation = method.getAnnotation(GamepadBind.class);

                if (annotation.actionType() == GamepadEx.ActionType.ON_TOGGLE) {
                    Class<?>[] params = method.getParameterTypes();
                    if (params.length != 1 || (params[0] != int.class && params[0] != Integer.class)) {
                        throw new IllegalArgumentException("Method " + method.getName() + " annotated with ON_TOGGLE must accept exactly one 'int' parameter.");
                    }
                } else {
                    if (method.getParameterCount() != 0) {
                        throw new IllegalArgumentException("Method " + method.getName() + " must have 0 parameters for non-toggle bindings.");
                    }
                }

                // 检查该绑定是否允许在当前 OpMode 中运行
                Class<?>[] allowedOpModes = annotation.activeIn();
                if (allowedOpModes.length > 0) {
                    boolean isAllowed = false;
                    for (Class<?> allowedClass : allowedOpModes) {
                        // 使用 isAssignableFrom 兼容继承关系 (如果当前 OpMode 继承自 allowedClass 也会放行)
                        if (allowedClass.isAssignableFrom(currentOpModeClass)) {
                            isAllowed = true;
                            break;
                        }
                    }
                    // 如果当前 OpMode 不在允许列表中，直接跳过这个方法的绑定！
                    if (!isAllowed) continue;
                }


                GamepadEx targetGp = (annotation.gamepad() == 1) ? gp1 : gp2;
                if (targetGp == null) continue;

                method.setAccessible(true);
                GamepadEx.ButtonHandler handler = targetGp.getHandler(annotation.button());
                boolean hasId = !annotation.id().isEmpty();

                try {
                    if (annotation.actionType() == GamepadEx.ActionType.ON_TOGGLE) {
                        Consumer<Integer> action = count -> {
                            try { method.invoke(target, count); }
                            catch (Exception e) { throw new RuntimeException(e); }
                        };
                        if (hasId) handler.onPress(annotation.id(), action);
                        else handler.onPress(action);
                    } else {
                        Runnable action = () -> {
                            try { method.invoke(target); }
                            catch (Exception e) { throw new RuntimeException(e); }
                        };
                        switch (annotation.actionType()) {
                            case ON_PRESS:   if(hasId) handler.onPress(annotation.id(), action);   else handler.onPress(action); break;
                            case ON_RELEASE: if(hasId) handler.onRelease(annotation.id(), action); else handler.onRelease(action); break;
                            case WHEN_DOWN:  if(hasId) handler.whenDown(annotation.id(), action);  else handler.whenDown(action); break;
                            case WHEN_UP:    if(hasId) handler.whenUp(annotation.id(), action);    else handler.whenUp(action); break;
                        }
                    }
                } catch (IllegalArgumentException e) {
                    throw new RuntimeException("Annotation param doesn't match" + method.getName(), e);
                }
            }
        }
    }
}
