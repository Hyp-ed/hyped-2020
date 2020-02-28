
LIT = ./utils/Lit/common/lit/lit.py
LIT_TEST_DIR = test/lit


lit:
	$(Verb) export ROOT=$(ROOT) ; \
	$(LIT) $(LIT_TEST_DIR)

lit-clean:
	$(Verb) $(RM) -r `find $(LIT_TEST_DIR) -name "Output" -type d -print`
