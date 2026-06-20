package org.firstinspires.ftc.teamcode.common.util;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * 为方法参数指定名称，供运行时反射获取。
 * <p>
 * Control Hub (Java 8 / Android API &lt; 26) 不支持
 * {@code java.lang.reflect.Parameter.getName()}，
 * 使用此注解可替代编译期 {@code -parameters} 标志。
 *
 * <pre>{@code
 * @AutoTask("moveTo")
 * public void moveTo(@Param("x") double x, @Param("y") double y) { ... }
 * }</pre>
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.PARAMETER)
public @interface Param {
    /** 参数名称 */
    String value();
}
