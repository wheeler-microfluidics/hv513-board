from path_helpers import path
try:
    from .node import (Proxy as _Proxy, I2cProxy as _I2cProxy,
                       SerialProxy as _SerialProxy)

    class ProxyMixin(object):
        '''
        Mixin class to add convenience wrappers around methods of the generated
        `node.Proxy` class.

        For example, expose config and state getters/setters as attributes.
        '''
        host_package_name = str(path(__file__).parent.name.replace('_', '-'))

        @property
        def config(self):
            from .config import Config

            return Config.FromString(self.serialize_config().tostring())

        @config.setter
        def config(self, value):
            return self.update_config(value)

        @property
        def state(self):
            from .config import State

            return State.FromString(self.serialize_state().tostring())

        @state.setter
        def state(self, value):
            return self.update_state(value)

        def update_config(self, **kwargs):
            '''
            Update fields in the config object based on keyword arguments.

            By default, these values will be saved to EEPROM. To prevent this
            (e.g., to verify system behavior before committing the changes),
            you can pass the special keyword argument 'save=False'. In this case,
            you will need to call the method save_config() to make your changes
            persistent.
            '''

            from .config import Config

            save = True
            if 'save' in kwargs.keys() and not kwargs.pop('save'):
                save = False

            config = Config(**kwargs)
            return_code = super(ProxyMixin, self).update_config(config)

            if save:
                super(ProxyMixin, self).save_config()

            return return_code

        def update_state(self, **kwargs):
            from .config import State

            state = State(**kwargs)
            return super(ProxyMixin, self).update_state(state)


    class Proxy(ProxyMixin, _Proxy):
        pass

    class I2cProxy(ProxyMixin, _I2cProxy):
        pass

    class SerialProxy(ProxyMixin, _SerialProxy):
        pass

except (ImportError, TypeError):
    Proxy = None
    I2cProxy = None
    SerialProxy = None
