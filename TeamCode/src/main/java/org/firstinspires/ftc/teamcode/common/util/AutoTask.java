package org.firstinspires.ftc.teamcode.common.util;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Marks a class or method as an HTTP-invokable task for {@link HttpJsonService}.
 * Can be placed on a class (all public methods become tasks) or individual methods.
 */
@Retention(RetentionPolicy.RUNTIME)
@Target({ElementType.TYPE, ElementType.METHOD})
public @interface AutoTask {
    /** Optional task name override. Defaults to method name. */
    String value() default "";
}
