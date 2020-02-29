
LIT          := ./utils/Lit/common/lit/lit.py
LIT_TEST_DIR := test/lit
LIT_ARGS     := -v
LIT_LIB      := utils/Lit
LIT_TAR      := $(LIBS_DIR)/lit.tar.gz
LIT_FILTER   :=


lit: $(LIT_LIB)
	$(Verb) export ROOT=$(ROOT) ; export PATH="utils/Lit/$(UNAME):$$PATH" ; \
	$(LIT) $(LIT_ARGS) --filter=$(LIT_FILTER) $(LIT_TEST_DIR)

lit-clean:
	$(Verb) $(RM) -r `find $(LIT_TEST_DIR) -name "Output" -type d -print`

$(LIT_LIB): $(LIT_TAR)
	$(Echo) Installing LIT
	$(Verb) tar -zxvf $<
	$(Verb) touch $@
